/*
 * GTK3 graphical interpreter for Trajedy, with some debugging features.
 */

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <numeric>
#include <deque>
#include <fstream>
#include <gmpxx.h>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <glibmm.h>
#include <gtkmm/application.h>
#include <gtkmm/box.h>
#include <gtkmm/button.h>
#include <gtkmm/entry.h>
#include <gtkmm/layout.h>
#include <gtkmm/paned.h>
#include <gtkmm/separator.h>
#include <gtkmm/scrolledwindow.h>
#include <gtkmm/textview.h>
#include <gtkmm/togglebutton.h>
#include <gtkmm/window.h>

#include "v2.hpp"
#include "interp.hpp"
#include "program_option.hpp"

// Draw program area.
struct ProgramArea: Gtk::Layout {
    DebugState debug_state;

    // Display parameters
    double square_size; // screen pixels
    double grid_line_px; // screen pixels

    std::string font_family;
    std::string newline_sym;
    double text_margin; // fraction of square

    double pointer_length; // fraction of square
    double path_line_px;
    std::size_t history_path_length;

    // should be provided by ScrolledWindow
    Glib::RefPtr <Gtk::Adjustment> hadjustment, vadjustment;

    ProgramArea(TState state,
                Glib::RefPtr <Gtk::Adjustment> hadjustment,
                Glib::RefPtr <Gtk::Adjustment> vadjustment):
        debug_state {state},
        square_size {32.0},
        grid_line_px {2.0},

        font_family {"monospace"},
        newline_sym {"↵"},
        text_margin {0.1},

        pointer_length {0.5},
        path_line_px {2.0},
        history_path_length {100},

        hadjustment {hadjustment},
        vadjustment {vadjustment}
    {
        zoom(square_size);
    }

    void zoom(double square_size_) {
        square_size = square_size_;
        // add one square_size for margins
        set_size(square_size * (debug_state.state.width() + 2),
                 square_size * (debug_state.state.height() + 2));
    }

    bool on_draw(const Cairo::RefPtr <Cairo::Context>& cairo) {
        cairo->set_source_rgb(0.0, 0.0, 0.0);
        cairo->paint();

        // Size each square to 1×1 in user space
        cairo->scale(square_size, square_size);
        cairo->translate(-hadjustment->get_value() / square_size + 1,
                         -vadjustment->get_value() / square_size + 1);

        // FIXME: we currently draw everything, even stuff outside the viewport.

        const size_t cols = debug_state.state.width();
        const size_t rows = debug_state.state.height();

        // Background for program area.
        cairo->set_source_rgb(1.0, 1.0, 1.0);
        cairo->move_to(0, 0);
        cairo->line_to(cols, 0);
        cairo->line_to(cols, rows);
        cairo->line_to(0, rows);
        cairo->close_path();
        cairo->fill();

        // Highlight current square.
        cairo->set_source_rgb(1.0, 1.0, 0.5);
        const v2 <long> current_square = debug_state.state.ptr.current_square;
        cairo->move_to(current_square.x, current_square.y);
        cairo->line_to(current_square.x + 1, current_square.y);
        cairo->line_to(current_square.x + 1, current_square.y + 1);
        cairo->line_to(current_square.x, current_square.y + 1);
        cairo->close_path();
        cairo->fill();

        // Grid lines.
        const double grid_line_width = grid_line_px / square_size;
        std::vector <double> dash_length = {5.0 * grid_line_width};
        cairo->set_line_cap(Cairo::LINE_CAP_SQUARE);
        cairo->set_dash(dash_length, 0.0);
        cairo->set_source_rgb(0.5, 0.5, 0.5);
        cairo->set_line_width(grid_line_width);
        for (std::size_t x = 0; x <= cols; ++x) {
            cairo->move_to(x, 0);
            cairo->line_to(x, rows);
            cairo->stroke();
        }
        for (std::size_t y = 0; y <= rows; ++y) {
            cairo->move_to(0, y);
            cairo->line_to(cols, y);
            cairo->stroke();
        }
        cairo->set_dash(std::vector <double> {}, 0.0);

        // Recent path.
        // We draw from least to most recent. While this doesn't matter now,
        // it would if we varied the path colour or opacity.
        if (!debug_state.undos.empty()) {
            std::size_t path_start =
                debug_state.undos.size() -
                std::min(debug_state.undos.size(), history_path_length);
            cairo->set_source_rgb(0.0, 0.0, 0.8);
            cairo->move_to(debug_state.undos[path_start].old_ptr.position.x.get_d(),
                           debug_state.undos[path_start].old_ptr.position.y.get_d());
            for (std::size_t i = path_start; i < debug_state.undos.size(); ++i) {
                cairo->line_to(debug_state.undos[i].old_ptr.position.x.get_d(),
                               debug_state.undos[i].old_ptr.position.y.get_d());
                if (debug_state.undos[i].was_mirror) {
                    cairo->line_to(debug_state.undos[i].mirror_hit.x.get_d(),
                                   debug_state.undos[i].mirror_hit.y.get_d());
                }
            }
            cairo->line_to(debug_state.state.ptr.position.x.get_d(),
                           debug_state.state.ptr.position.y.get_d());
            cairo->stroke();
        }

        auto highlight_around_square = [&](long col, long row) {
            cairo->set_line_width(grid_line_width);
            cairo->move_to(col + grid_line_width, row + grid_line_width);
            cairo->line_to(col + grid_line_width, row + 1 - grid_line_width);
            cairo->line_to(col + 1 - grid_line_width, row + 1 - grid_line_width);
            cairo->line_to(col + 1 - grid_line_width, row + grid_line_width);
            cairo->close_path();
            cairo->stroke();
        };

        // Extra information for current pointer.
        v2 <double> pointer_pos {debug_state.state.ptr.position.x.get_d(),
                                 debug_state.state.ptr.position.y.get_d()};
        auto& state = debug_state.state;
        if (state.pointer_in_range() && state.mode == TState::NormalMode) {
            TChar c = state.current_square();
            if (c == TState::comma || c == TState::period) {
                // Show target square
                if (!state.ptr.is_along_edge()) {
                    TPointer ptr2 = state.ptr;
                    ++ptr2;
                    cairo->set_source_rgb(0.5, 0.5, 0.5);
                    highlight_around_square(ptr2.current_square.x, ptr2.current_square.y);
                }
            } else if (c == TState::mirror1 || c == TState::mirror2) {
                // Nothing to do
            } else if (c == TState::space) {
            } else {
                // A beacon; show (dashed) line to target beacon
                TPointer ptr0 = state.ptr;
                auto target = state.turn_beacon(c);
                // That should have been a no-op
                //assert (ptr0 == state.ptr);
                if (std::get<1>(target) != v2 <long> {-1, -1}) {
                    cairo->set_source_rgb(0.8, 0.8, 0.0);
                    cairo->set_dash(dash_length, 0.0);
                    cairo->move_to(pointer_pos.x, pointer_pos.y);
                    cairo->line_to(std::get<0>(target).x.get_d(),
                                   std::get<0>(target).y.get_d());
                    cairo->stroke();
                    cairo->set_dash(std::vector <double> {}, 0.0);
                }
            }
        }

        // Characters in the program area.
        // This is drawn over the program path so that the
        // characters do not get too obscured.
        cairo->set_font_size(1.0 - 2 * text_margin);
        cairo->set_line_width(2.0 / square_size); // mirrors
        cairo->set_source_rgb(0.0, 0.0, 0.0);
        Cairo::FontExtents extents;
        cairo->get_font_extents(extents);
        for (std::size_t y = 0; y < rows; ++y) {
            for (std::size_t x = 0; x < cols; ++x) {
                std::string ch = { debug_state.state.squares[y][x] };
                // Special characters.
                if (ch[0] == TState::mirror1) {
                    cairo->move_to(x, y + 1);
                    cairo->line_to(x + 1, y);
                    cairo->stroke();
                } else if (ch[0] == TState::mirror2) {
                    cairo->move_to(x, y);
                    cairo->line_to(x + 1, y + 1);
                    cairo->stroke();
                } else {
                    if (ch[0] == TState::newline) {
                        ch = newline_sym;
                    } else if (ch[0] == TState::space) {
                        // get_text_extents doesn't work for whitespace, just skip it
                        continue;
                    }
                    cairo->move_to(x + text_margin,
                                   y + 1 - text_margin - extents.descent);
                    cairo->show_text(ch);
                    cairo->fill();
                }
            }
        }

        // Program pointer.
        if (debug_state.state.ptr.is_along_edge()) {
            // Bad pointer
            cairo->set_source_rgb(0.8, 0.2, 0.0);
        } else if (!debug_state.state.pointer_in_range()) {
            // Pointer outside program area
            cairo->set_source_rgb(0.5, 0.5, 0.5);
        } else {
            cairo->set_source_rgb(0.2, 0.5, 0.0);
        }
        cairo->begin_new_sub_path();
        cairo->arc(pointer_pos.x, pointer_pos.y,
                   0.1, 0.0, 2 * M_PI);
        cairo->fill();

        // pre-scale by L1 norm to avoid overflow
        mpz_class dir_scale = (abs(debug_state.state.ptr.direction.x) +
                               abs(debug_state.state.ptr.direction.y));
        v2 <double> pointer_dir {mpq_class {debug_state.state.ptr.direction.x, dir_scale}.get_d(),
                                 mpq_class {debug_state.state.ptr.direction.y, dir_scale}.get_d()};
        double norm = std::sqrt(pointer_dir.norm2());
        pointer_dir *= v2 <double> {pointer_length / norm};

        v2 <double> pointer_tip = pointer_pos + pointer_dir;
        v2 <double> arrow_start = pointer_pos + v2 <double> {0.5} * pointer_dir;
        v2 <double> arrow_width = v2 <double> {0.25} * v2 <double> {pointer_dir.y, -pointer_dir.x};
        v2 <double> arrow_left = arrow_start + arrow_width, arrow_right = arrow_start - arrow_width;
        cairo->move_to(pointer_pos.x, pointer_pos.y);
        cairo->line_to(pointer_tip.x, pointer_tip.y);
        cairo->stroke();

        cairo->move_to(arrow_left.x, arrow_left.y);
        cairo->line_to(arrow_right.x, arrow_right.y);
        cairo->line_to(pointer_tip.x, pointer_tip.y);
        cairo->close_path();
        cairo->fill();

        return true;
    }
};

struct MainWindow: Gtk::Window {
    /*
     * Layout:
     * The window is split into a control panel (left) and display (right).
     * The control panel steps the program and adjusts settings.
     * The display area shows the program area (top) and I/O buffers (bottom).
     *
     * +----------+-------------------------+
     * |          | |P|r|o|g|r|a|m| |a|r|e|a|
     * | controls |                         |
     * |          |-------------------------|
     * |          | input buffer            |
     * |          |-------------------------|
     * |          | output buffer           |
     * +----------+-------------------------+
     */
    Gtk::Paned main_panels;
    Gtk::Box control_panels;
    Gtk::Paned display_panels;
    Gtk::Paned buffer_panels;

    Gtk::Box control_bar;
    // dynamic label, set by update_step_counter
    Gtk::Label control_bar_label;
    Gtk::Button reset;
    Gtk::Button undo;
    Gtk::ToggleButton stop;
    Gtk::Button step;
    Gtk::ToggleButton run;
    Gtk::Separator control_bar_hsep;

    double run_delay; // seconds
    Gtk::Box run_delay_bar;
    Gtk::Label run_delay_label;
    Gtk::Entry run_delay_entry; // display in milliseconds
    Gtk::Label run_delay_unit;

    // NB: needed by program_area constructor
    Gtk::ScrolledWindow program_area_scroll;
    ProgramArea program_area;
    TState state0; // reset target

    Gtk::Box input_view_box;
    Gtk::Label input_view_label;
    Gtk::ScrolledWindow input_view_scroll;
    Gtk::TextView input_view;
    Gtk::Box output_view_box;
    Gtk::Label output_view_label;
    Gtk::ScrolledWindow output_view_scroll;
    Gtk::TextView output_view;
    Glib::RefPtr <Gtk::TextBuffer::Tag> greyed_out_tag;
    // Handle to disable callbacks when editing the buffer
    sigc::connection input_view_callback;

    // Helpers for updating input_view and output_view after state changes
    std::function <void(void)> update_input_buffer, update_output_buffer;
    // Helpers to step the program and update the GUI
    std::function <bool(void)> step_program, unstep_program;
    // Handle to stop running program in background
    sigc::connection background_runner;

    std::function <void(void)> update_step_counter; // updates control_bar_label

    MainWindow(TState state):
        main_panels {Gtk::ORIENTATION_HORIZONTAL},
        control_panels {Gtk::ORIENTATION_VERTICAL, 5},
        display_panels {Gtk::ORIENTATION_VERTICAL},
        buffer_panels {Gtk::ORIENTATION_VERTICAL},

        control_bar {Gtk::ORIENTATION_HORIZONTAL, 5},
        control_bar_label {"Program steps: 0"},
        reset {"«"},
        undo {"◀"},
        stop {"■"},
        step {"▶"},
        // ⏩ is better, but may not be in the font
        run {"»"},

        run_delay {0.0},
        run_delay_bar {Gtk::ORIENTATION_HORIZONTAL, 5},
        run_delay_label {"Step delay"},
        run_delay_unit {"ms"},

        program_area {
            state,
            program_area_scroll.get_hadjustment(),
            program_area_scroll.get_vadjustment() },
        state0 {state},

        input_view_box {Gtk::ORIENTATION_VERTICAL},
        input_view_label {"Input:"},
        output_view_box {Gtk::ORIENTATION_VERTICAL},
        output_view_label {"Output:"},
        output_view {Gtk::TextBuffer::create(input_view.get_buffer()->get_tag_table())}
    {
        // Panel layout
        main_panels.pack1(control_panels, Gtk::SHRINK);
        main_panels.pack2(display_panels, Gtk::EXPAND);
        main_panels.property_wide_handle() = true;

        update_input_buffer = [&]() {
            // Don't run the input_view handler (which would
            // modify debug_state from under us)
            input_view_callback.block();

            auto buffer = input_view.get_buffer();
            buffer->begin_user_action();
            buffer->erase(buffer->begin(), buffer->end());
            // past_input
            buffer->insert_with_tag
                (buffer->end(),
                 Glib::ustring {
                    program_area.debug_state.past_input.begin(),
                    program_area.debug_state.past_input.end() },
                 greyed_out_tag);

            // Remaining input
            buffer->insert(buffer->end(),
                           Glib::ustring {program_area.debug_state.input.begin(),
                                          program_area.debug_state.input.end()});
            buffer->end_user_action();

            input_view_callback.unblock();
        };
        update_output_buffer = [&]() {
            auto buffer = output_view.get_buffer();
            buffer->begin_user_action();
            buffer->erase(buffer->begin(), buffer->end());
            buffer->insert(buffer->end(),
                           Glib::ustring {program_area.debug_state.output.begin(),
                                          program_area.debug_state.output.end()});

            buffer->insert_with_tag
                (buffer->end(),
                 Glib::ustring {
                    program_area.debug_state.future_output.begin(),
                    program_area.debug_state.future_output.end() },
                 greyed_out_tag);
            buffer->end_user_action();
        };
        update_step_counter = [&]() {
            control_bar_label.set_text(
                Glib::ustring::compose
                    ("Program steps: %1",
                     Glib::ustring::format(program_area.debug_state.undos.size())));
        };

        step_program = [&]() -> bool {
            if (program_area.debug_state.state.ptr.is_along_edge() ||
                (!program_area.debug_state.state.pointer_in_range() &&
                 // check that we aren't at the initial step
                 !program_area.debug_state.undos.empty())) {
                // nothing more to run
                stop.set_active();
                return false;
            }
            std::size_t input_size_was = program_area.debug_state.past_input.size();
            std::size_t output_size_was = program_area.debug_state.output.size();
            program_area.debug_state.step();
            if (input_size_was != program_area.debug_state.past_input.size()) {
                update_input_buffer();
            }
            if (output_size_was != program_area.debug_state.output.size()) {
                update_output_buffer();
            }
            update_step_counter();
            program_area.queue_draw();
            return true;
        };

        unstep_program = [&]() -> bool {
            if (program_area.debug_state.undos.empty()) {
                return false;
            }
            std::size_t input_size_was = program_area.debug_state.past_input.size();
            std::size_t output_size_was = program_area.debug_state.output.size();
            program_area.debug_state.unstep();
            if (input_size_was != program_area.debug_state.past_input.size()) {
                update_input_buffer();
            }
            if (output_size_was != program_area.debug_state.output.size()) {
                update_output_buffer();
            }
            update_step_counter();
            program_area.queue_draw();
            return true;
        };

        // Control bar
        control_panels.pack_start(control_bar_label, Gtk::PACK_SHRINK);
        control_panels.pack_start(control_bar, Gtk::PACK_SHRINK);
        reset.set_tooltip_text("Reset");
        reset.signal_clicked().connect
            ([&]() {
                // This should be more-or-less like undoing everything, but faster
                program_area.debug_state.undos.clear();

                program_area.debug_state.input.insert
                    (program_area.debug_state.input.begin(),
                     program_area.debug_state.past_input.begin(),
                     program_area.debug_state.past_input.end());
                program_area.debug_state.past_input.clear();

                program_area.debug_state.future_output.insert
                    (program_area.debug_state.future_output.begin(),
                     program_area.debug_state.output.begin(),
                     program_area.debug_state.output.end());
                program_area.debug_state.output.clear();

                program_area.debug_state.state = state0;

                update_input_buffer();
                update_output_buffer();
                update_step_counter();
                program_area.queue_draw();

                // ensure paused
                stop.set_active();
            });
        control_bar.pack_start(reset);
                
        undo.set_tooltip_text("Undo");
        undo.signal_clicked().connect
            ([&]() {
                unstep_program();
                // now pause
                stop.set_active();
            });
        control_bar.pack_start(undo);

        stop.set_tooltip_text("Stop program");
        stop.signal_clicked().connect
            ([&]() {
                if (stop.get_active() == run.get_active()) {
                    // This will trigger run->signal_clicked.
                    // The preceding condition avoids recursion.
                    run.set_active(!stop.get_active());
                    if (stop.get_active()) {
                        background_runner.disconnect();
                    } else {
                        if (run_delay == 0) {
                            background_runner = Glib::signal_idle().connect(step_program);
                        } else {
                            background_runner =
                                Glib::signal_timeout().connect(step_program, run_delay * 1000);
                        }
                    }
                }
            });
        stop.set_active();
        control_bar.pack_start(stop);

        step.set_tooltip_text("Run one step");
        step.signal_clicked().connect
            ([&]() {
                step_program();
                // now pause
                stop.set_active();
            });
        control_bar.pack_start(step);

        run.set_tooltip_text("Run program");
        run.signal_clicked().connect
            ([&]() {
                if (run.get_active() == stop.get_active()) {
                    // This will trigger stop->signal_clicked.
                    // The preceding condition avoids recursion.
                    stop.set_active(!run.get_active());
                    if (run.get_active()) {
                        if (run_delay == 0) {
                            background_runner = Glib::signal_idle().connect(step_program);
                        } else {
                            background_runner =
                                Glib::signal_timeout().connect(step_program, run_delay * 1000);
                        }
                    } else {
                        background_runner.disconnect();
                    }
                }
            });
        control_bar.pack_start(run);
        control_panels.pack_start(control_bar_hsep, Gtk::PACK_SHRINK);

        // Run delay bar
        run_delay_bar.pack_start(run_delay_label, Gtk::PACK_SHRINK);
        run_delay_bar.pack_start(run_delay_entry);
        run_delay_entry.set_alignment(1.0); // align right
        run_delay_entry.set_input_purpose(Gtk::INPUT_PURPOSE_NUMBER);
        run_delay_bar.pack_start(run_delay_unit, Gtk::PACK_SHRINK);
        control_panels.pack_start(run_delay_bar, Gtk::PACK_SHRINK);

        run_delay_entry.set_text(Glib::ustring::format(run_delay * 1000));
        auto update_run_delay = [&]() {
            // FIXME: this suppresses a pango assertion failure, but why?
            run_delay_entry.set_position(0);
            // check value
            std::size_t pos;
            try {
                double val = std::stod(run_delay_entry.get_text().raw(), &pos);
                const double sensible_upper_limit = 10e3; // 10s
                if (val >= 0 && val <= sensible_upper_limit) {
                    run_delay = val / 1000;
                    if (run.get_active()) {
                        // Adjust program's step scheduler
                        background_runner.disconnect();
                        if (run_delay == 0) {
                            background_runner = Glib::signal_idle().connect(step_program);
                        } else {
                            background_runner =
                                Glib::signal_timeout().connect(step_program, run_delay * 1000);
                        }
                    }
                    // done
                    return;
                }
            } catch(const std::invalid_argument&) {
            } catch(const std::out_of_range&) {
            }
            // invalid value; reset it
            run_delay_entry.set_text(Glib::ustring::format(run_delay * 1000));
        };
        run_delay_entry.signal_focus_out_event().connect
            ([&, update_run_delay](GdkEventFocus*) -> bool {
                update_run_delay();
                return true;
            });
        run_delay_entry.signal_activate().connect(update_run_delay);

        // Display panel
        display_panels.pack1(program_area_scroll, Gtk::EXPAND);
        program_area_scroll.add(program_area);

        // program_area has no default size, so give it maximum height by default
        display_panels.property_position() =
            display_panels.property_max_position();
        display_panels.pack2(buffer_panels, Gtk::EXPAND);
        display_panels.property_wide_handle() = true;

        input_view_label.property_xalign() = 0.1;
        input_view_box.pack_start(input_view_label, Gtk::PACK_SHRINK);
        input_view_box.pack_start(input_view_scroll);
        input_view_scroll.add(input_view);
        output_view_label.property_xalign() = 0.1;
        output_view_box.pack_start(output_view_label, Gtk::PACK_SHRINK);
        output_view_box.pack_start(output_view_scroll);
        output_view_scroll.add(output_view);

        // This tag indicates past_input and future_output.
        // It also blocks editing of past_input in input_view.
        greyed_out_tag = input_view.get_buffer()->create_tag();
        //greyed_out_tag->property_foreground_set() = true;
        greyed_out_tag->property_foreground() = Glib::ustring {"#999999"};
        greyed_out_tag->property_editable() = false;
        // Just copy everything on each edit.
        // If we want to be more efficient we have to keep track
        // of the cursor, or use the GTK text buffer directly.
        input_view_callback = input_view.get_buffer()->signal_changed().connect
            ([&]() {
                auto buffer = input_view.get_buffer();
                auto begin = buffer->begin(), end = buffer->end();
                if (begin != end && begin.starts_tag(greyed_out_tag)) {
                    while (begin != end && !begin.ends_tag(greyed_out_tag)) {
                        ++begin;
                    }
                }
                program_area.debug_state.input.clear();
                Glib::ustring new_input = buffer->get_text(begin, end);
                program_area.debug_state.input.insert(program_area.debug_state.input.begin(),
                                                      new_input.begin(), new_input.end());

                // Invalidate all of future_output. This is very conservative
                // but we don't know which of it is still valid.
                program_area.debug_state.future_output.clear();
                update_output_buffer();
            });
        // Prevent adding text in front of past_input's greyed_out_tag.
        input_view.get_buffer()->signal_insert().connect
            ([&](const Gtk::TextBuffer::iterator& pos, const Glib::ustring& text, int bytes) {
                auto buffer = input_view.get_buffer();
                if (pos == buffer->begin()) {
                    if (pos.starts_tag(greyed_out_tag)) {
                        input_view.get_buffer()->signal_insert().emission_stop();
                    }
                }
            }, false);
        output_view.set_editable(false);

        buffer_panels.pack1(input_view_box, Gtk::EXPAND);
        buffer_panels.pack2(output_view_box, Gtk::EXPAND);
        buffer_panels.property_wide_handle() = true;

        add(main_panels);
        show_all_children();
    }
};

int main(int argc, char** argv) {
    auto gtkApp = Gtk::Application::create(argc, argv, "",
                      Gio::APPLICATION_NON_UNIQUE | Gio::APPLICATION_HANDLES_COMMAND_LINE);

    std::basic_istream <TChar>* prog_file = &std::cin;
    std::string prog_filename = "-";
    std::basic_istream <TChar>* input_file = nullptr;
    std::string input_filename = "";
    option_parser opt_parser;
    opt_parser
        ('h', "help", "show this help text",
         [&opt_parser, argv](){
            std::cout << "Usage: " << argv[0] << opt_parser.usage() << "\n";
            std::cout << opt_parser.description() << "\n";
            std::exit(0);
        })

        .defaults_now_optional()
        ("PROG.tra", "program file (or - for standard input)",
         [&prog_filename](char const* file) {
            prog_filename = file;
        })

        ("input-file", "input file (or - for standard input)",
         [&input_filename](char const* file) {
            input_filename = file;
        })

        ("", "",
         [](char const* arg) {
            throw option_parser::parse_error(std::string("too many arguments: ") + arg, true);
        })
        ;

    std::vector <option_parser::parse_error> errs = opt_parser.parse_argv(argc, argv);
    for (auto err : errs) {
        if (err.opt_name.empty()) {
            std::cerr << "Error parsing command line: " << err.message << "\n";
        } else {
            std::cerr << "Error parsing " << err.opt_name << ": " << err.message << "\n";
        }
        if (err.fatal) {
            std::exit(1);
        }
    }

    if (prog_filename != "-") {
        prog_file = new std::basic_ifstream <TChar> {prog_filename};
        if (!*prog_file) {
            std::cerr << "Failed to open file: " << prog_filename << "\n";
            std::exit(1);
        }
    }

    if (input_filename == "") {
        // no input
    } else if (input_filename == "-") {
        input_file = &std::cin;
    } else {
        input_file = new std::basic_ifstream <TChar> {input_filename};
        if (!*input_file) {
            std::cerr << "Failed to open file: " << input_filename << "\n";
            std::exit(1);
        }
    }

    std::basic_string <TChar> prog_text;
    {
        TChar c;
        while (prog_file->get(c)) {
            prog_text += c;
        }
    }
    TState state(prog_text);

    std::string input_text;
    if (input_file) {
        TChar c;
        while (input_file->get(c)) {
            input_text += c;
        }
    }

    MainWindow window(state);
    window.set_title("Trajedy");
    window.set_default_size(500, 500);
    window.program_area.show();
    window.input_view.get_buffer()->insert_at_cursor(input_text);

    gtkApp->signal_command_line().connect(
        [&](const Glib::RefPtr<Gio::ApplicationCommandLine>& cmd)->int {
            gtkApp->activate();
            return 0;
        }, false);

    return gtkApp->run(window);
}
