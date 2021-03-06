/*
 * GTK3 graphical interpreter for Trajedy, with some debugging features.
 */

#include <algorithm>
#include <cassert>
#include <codecvt>
#include <cstdlib>
#include <numeric>
#include <deque>
#include <fstream>
#include <gmpxx.h>
#include <iomanip>
#include <iostream>
#include <locale>
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
#include <gtkmm/radiobutton.h>
#include <gtkmm/separator.h>
#include <gtkmm/scrolledwindow.h>
#include <gtkmm/textview.h>
#include <gtkmm/togglebutton.h>
#include <gtkmm/window.h>

#include "v2.hpp"
#include "interp.hpp"
#include "program_option.hpp"

// Draw program area.
struct ProgramView: Gtk::Layout {
    DebugState debug_state;

    // Display parameters
    double square_size; // screen pixels
    double grid_line_px; // screen pixels

    std::string font_family;
    Glib::ustring newline_sym;
    Glib::ustring EOI_sym;
    double text_margin; // fraction of square

    double pointer_length; // fraction of square
    double path_line_px;
    std::size_t history_path_length;

    // Should be provided by containing ScrolledWindow
    Glib::RefPtr <Gtk::Adjustment> hadjustment, vadjustment;

    // Cached
    // FIXME: handle encoding errors
    std::wstring_convert <std::codecvt_utf8 <TChar>, TChar> utf8_convert;

    ProgramView(TState state,
                Glib::RefPtr <Gtk::Adjustment> hadjustment,
                Glib::RefPtr <Gtk::Adjustment> vadjustment):
        debug_state {state},
        square_size {32.0},
        grid_line_px {2.0},

        // monospace isn't necessary, but makes sizing easier
        font_family {"monospace"},
        newline_sym {"↵"},
        EOI_sym {"END"},
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
        if (state.pointer_in_range()) {
            TChar c = state.current_square();
            if (state.mode == TState::NormalMode) {
                if (c == TState::comma || c == TState::period || c == TState::question_mark) {
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
                    state.ptr = ptr0;
                    const v2 <long>& beacon = std::get<1>(target);
                    if (beacon != v2 <long> {-1, -1}) {
                        cairo->set_source_rgb(0.8, 0.8, 0.0);
                        cairo->set_dash(dash_length, 0.0);
                        cairo->move_to(pointer_pos.x, pointer_pos.y);
                        cairo->line_to(std::get<0>(target).x.get_d(),
                                       std::get<0>(target).y.get_d());
                        cairo->stroke();
                        cairo->set_dash(std::vector <double> {}, 0.0);
                        highlight_around_square(beacon.x, beacon.y);
                    }
                }
            } else if (state.mode == TState::SpecialCharMode) {
                if (c == TState::comma || c == TState::period ||
                    c == TState::question_mark || c == TState::EOI) {
                    // Show target square
                    TPointer ptr0 = state.ptr;
                    TChar c_target = 0;
                    if (c == TState::comma) {
                        c_target = TState::comma_target;
                    } else if (c == TState::period) {
                        c_target = TState::period_target;
                    } else if (c == TState::question_mark) {
                        c_target = TState::question_target;
                    } else if (c == TState::EOI) {
                        c_target = TState::EOI_target;
                    }
                    if (c_target) {
                        auto target = state.turn_beacon(c_target);
                        state.ptr = ptr0;
                        const v2 <long>& beacon = std::get<1>(target);
                        if (beacon != v2 <long> {-1, -1}) {
                            cairo->set_source_rgb(0.8, 0.8, 0.0);
                            cairo->set_dash(dash_length, 0.0);
                            cairo->move_to(pointer_pos.x, pointer_pos.y);
                            cairo->line_to(std::get<0>(target).x.get_d(),
                                           std::get<0>(target).y.get_d());
                            cairo->stroke();
                            cairo->set_dash(std::vector <double> {}, 0.0);
                            highlight_around_square(beacon.x, beacon.y);
                        }
                    }
                }
            }
        }

        // Characters in the program area.
        // This is drawn over the program path so that the
        // characters do not get too obscured.
        const double target_font_size = 1.0 - 2 * text_margin;
        cairo->set_font_size(target_font_size);
        cairo->set_line_width(2.0 / square_size); // mirrors
        cairo->set_source_rgb(0.0, 0.0, 0.0);
        Cairo::FontExtents font_extents;
        cairo->get_font_extents(font_extents);
        for (std::size_t y = 0; y < rows; ++y) {
            for (std::size_t x = 0; x < cols; ++x) {
                TChar c {debug_state.state.squares[y][x]};
                // Special characters.
                if (c == TState::mirror1) {
                    cairo->move_to(x, y + 1);
                    cairo->line_to(x + 1, y);
                    cairo->stroke();
                } else if (c == TState::mirror2) {
                    cairo->move_to(x, y);
                    cairo->line_to(x + 1, y + 1);
                    cairo->stroke();
                } else {
                    Glib::ustring c_utf8;
                    if (c == TState::newline) {
                        c_utf8 = newline_sym;
                    } else if (c == TState::space) {
                        // get_text_extents doesn't work for whitespace, just skip it
                        continue;
                    } else if (c == TState::EOI) {
                        c_utf8 = EOI_sym;
                    } else {
                        c_utf8 = utf8_convert.to_bytes(c);
                    }
                    // Center the character horizontally, scaling to fit
                    Cairo::TextExtents text_extents;
                    cairo->get_text_extents(c_utf8, text_extents);
                    double orig_width = text_extents.width - text_extents.x_bearing;
                    double x_padding = (target_font_size - orig_width) / 2;
                    if (orig_width > target_font_size) {
                        cairo->set_font_size(target_font_size * target_font_size / orig_width);
                        x_padding = 0;
                    }
                    cairo->move_to(x + text_margin + x_padding,
                                   y + 1 - text_margin - font_extents.descent);
                    cairo->show_text(c_utf8);
                    cairo->fill();
                    cairo->set_font_size(target_font_size);
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
     * It also displays debugging information, such as pointer coords.
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

    // Stop/run controls.
    Gtk::Box control_bar;
    // dynamic label, set by update_step_counter
    Gtk::Label control_bar_label;
    Gtk::Button reset;
    Gtk::Button undo;
    Gtk::ToggleButton stop;
    Gtk::Button step;
    Gtk::ToggleButton run;
    Gtk::Separator control_bar_hsep;

    // Control running speed.
    double run_delay; // seconds
    Gtk::Box run_delay_bar;
    Gtk::Label run_delay_label;
    Gtk::Entry run_delay_entry; // display in milliseconds
    Gtk::Label run_delay_unit;
    Gtk::Separator run_delay_hsep;

    // Show/set pointer coordinates.
    Gtk::Box pointer_info_mode_bar;
    Gtk::Label pointer_info_mode_label;
    // Fractions; mixed fractions (most useful); decimal approximations
    Gtk::RadioButtonGroup pointer_info_mode_button_group;
    Gtk::RadioButton pointer_info_mode_fullfrac;
    Gtk::RadioButton pointer_info_mode_mixedfrac;
    Gtk::RadioButton pointer_info_mode_decimal;
    Gtk::Box pointer_position_bar;
    Gtk::Label pointer_position_label;
    Gtk::Entry pointer_x_field, pointer_y_field;
    Gtk::Box pointer_direction_bar;
    Gtk::Label pointer_direction_label;
    Gtk::Entry pointer_dx_field, pointer_dy_field;

    // NB: needed by program_area constructor
    Gtk::ScrolledWindow program_area_scroll;
    ProgramView program_area;
    TState state0; // reset target

    // Input/output buffers.
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
    std::function <void(void)> update_pointer_info;
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

        pointer_info_mode_bar {Gtk::ORIENTATION_HORIZONTAL, 5},
        pointer_info_mode_label {"Value format:"},
        pointer_info_mode_fullfrac  {pointer_info_mode_button_group, "fraction"},
        pointer_info_mode_mixedfrac {pointer_info_mode_button_group, "offset"},
        pointer_info_mode_decimal   {pointer_info_mode_button_group, "decimal"},

        pointer_position_bar {Gtk::ORIENTATION_HORIZONTAL, 5},
        pointer_position_label {"Position"},
        pointer_direction_bar {Gtk::ORIENTATION_HORIZONTAL, 5},
        pointer_direction_label {"Direction"},

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
                    program_area.utf8_convert.to_bytes(
                        std::basic_string <TChar> {
                            program_area.debug_state.past_input.begin(),
                            program_area.debug_state.past_input.end() })
                    },
                 greyed_out_tag);

            // Remaining input
            buffer->insert(buffer->end(),
                           Glib::ustring {
                               program_area.utf8_convert.to_bytes(
                                   std::basic_string <TChar> {
                                       program_area.debug_state.input.begin(),
                                       program_area.debug_state.input.end() })
                           });
            buffer->end_user_action();

            input_view_callback.unblock();
        };
        update_output_buffer = [&]() {
            auto buffer = output_view.get_buffer();
            buffer->begin_user_action();
            buffer->erase(buffer->begin(), buffer->end());
            buffer->insert(buffer->end(),
                           Glib::ustring {
                               program_area.utf8_convert.to_bytes(
                                   std::basic_string <TChar> {
                                       program_area.debug_state.output.begin(),
                                       program_area.debug_state.output.end() })
                           });

            buffer->insert_with_tag
                (buffer->end(),
                 Glib::ustring {
                     program_area.utf8_convert.to_bytes(
                         std::basic_string <TChar> {
                             program_area.debug_state.future_output.begin(),
                             program_area.debug_state.future_output.end() })
                 },
                 greyed_out_tag);
            buffer->end_user_action();
        };
        update_step_counter = [&]() {
            control_bar_label.set_text(
                Glib::ustring::compose
                    ("Program steps: %1",
                     Glib::ustring::format(program_area.debug_state.undos.size())));
        };

        update_pointer_info = [&]() -> void {
            const TPointer& ptr = program_area.debug_state.state.ptr;
            CoordFormat fmt = CoordFormat::MixedFrac;
            if (pointer_info_mode_fullfrac.get_active()) {
                fmt = CoordFormat::FullFrac;
            } else if (pointer_info_mode_mixedfrac.get_active()) {
                fmt = CoordFormat::MixedFrac;
            } else if (pointer_info_mode_decimal.get_active()) {
                fmt = CoordFormat::Decimal;
            }
            pointer_x_field.set_text(print_coordinate(ptr.position.x, fmt));
            pointer_y_field.set_text(print_coordinate(ptr.position.y, fmt));
            pointer_dx_field.set_text(print_coordinate(ptr.direction.x, fmt));
            pointer_dy_field.set_text(print_coordinate(ptr.direction.y, fmt));
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
            update_pointer_info();
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
            update_pointer_info();
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
                update_pointer_info();
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
        control_panels.pack_start(run_delay_hsep, Gtk::PACK_SHRINK);

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


        // Pointer coordinates
        pointer_info_mode_bar.pack_start(pointer_info_mode_label, Gtk::PACK_SHRINK);
        pointer_info_mode_bar.pack_start(pointer_info_mode_fullfrac, Gtk::PACK_SHRINK);
        pointer_info_mode_bar.pack_start(pointer_info_mode_mixedfrac, Gtk::PACK_SHRINK);
        pointer_info_mode_bar.pack_start(pointer_info_mode_decimal, Gtk::PACK_SHRINK);
        control_panels.pack_start(pointer_info_mode_bar, Gtk::PACK_SHRINK);

        pointer_position_bar.pack_start(pointer_position_label, Gtk::PACK_SHRINK);
        pointer_position_bar.pack_start(pointer_x_field, Gtk::PACK_EXPAND_WIDGET);
        pointer_position_bar.pack_start(pointer_y_field, Gtk::PACK_EXPAND_WIDGET);
        control_panels.pack_start(pointer_position_bar, Gtk::PACK_SHRINK);
        pointer_x_field.set_width_chars(8);
        pointer_y_field.set_width_chars(8);

        pointer_direction_bar.pack_start(pointer_direction_label, Gtk::PACK_SHRINK);
        pointer_direction_bar.pack_start(pointer_dx_field, Gtk::PACK_EXPAND_WIDGET);
        pointer_direction_bar.pack_start(pointer_dy_field, Gtk::PACK_EXPAND_WIDGET);
        control_panels.pack_start(pointer_direction_bar, Gtk::PACK_SHRINK);
        pointer_dx_field.set_width_chars(8);
        pointer_dy_field.set_width_chars(8);

        pointer_info_mode_fullfrac.signal_clicked().connect
            ([&]() {
                update_pointer_info();
            });
        pointer_info_mode_mixedfrac.signal_clicked().connect
            ([&]() {
                update_pointer_info();
            });
        pointer_info_mode_decimal.signal_clicked().connect
            ([&]() {
                update_pointer_info();
            });
        // populate
        pointer_info_mode_mixedfrac.set_active();

        // TODO
        //pointer_x_field.signal_activate().connect()

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
                std::basic_string <TChar> input_chars =
                    program_area.utf8_convert.from_bytes(new_input.raw());
                program_area.debug_state.input.insert(program_area.debug_state.input.end(),
                                                      input_chars.begin(), input_chars.end());

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

    // Helpers for dealing with pointer coordinates
    enum class CoordFormat {
        FullFrac,
        MixedFrac,
        Decimal,
    };
    static Glib::ustring print_coordinate(const mpq_class& x, CoordFormat format) {
        switch (format) {
        case CoordFormat::FullFrac:
            return x.get_num().get_str() + "/" + x.get_den().get_str();
        case CoordFormat::MixedFrac:
            {
                mpz_class& num = const_cast <mpq_class&> (x).get_num();
                mpz_class& den = const_cast <mpq_class&> (x).get_den();
                mpz_class quot, rem;
                mpz_tdiv_qr(quot.get_mpz_t(), rem.get_mpz_t(), num.get_mpz_t(), den.get_mpz_t());
                Glib::ustring s = quot.get_str();
                if (rem != 0) {
                    mpz_abs(rem.get_mpz_t(), rem.get_mpz_t());
                    s += ' ';
                    s += rem.get_str();
                    s += '/';
                    s += den.get_str();
                }
                return s;
            }
        case CoordFormat::Decimal:
            {
                // TODO: use scientific notation
                const unsigned digits = 6; // TODO make this a parameter
                mpz_class& num = const_cast <mpq_class&> (x).get_num();
                mpz_class& den = const_cast <mpq_class&> (x).get_den();
                mpz_class quot = 10, rem;
                mpz_pow_ui(quot.get_mpz_t(), quot.get_mpz_t(), digits);
                quot *= num;
                mpz_tdiv_qr(quot.get_mpz_t(), rem.get_mpz_t(), quot.get_mpz_t(), den.get_mpz_t());
                Glib::ustring s = Glib::ustring::format
                    (std::setfill(L'0'), std::setw(digits + 1), quot.get_str());
                s.insert(s.size() - digits, ".");
                if (rem != 0) {
                    s += "...";
                }
                return s;
            }
        default:
            assert (false);
        }
    }
    static bool parse_coordinate(Glib::ustring s, mpq_class& x) {
        if (!s.is_ascii()) {
            return false;
        }
        try {
            if (s.find('.') != Glib::ustring::npos) {
                // assume decimal
                // strip trailing space
                while (!s.empty() && s[s.size() - 1] == ' ') {
                    s.erase(s.size() - 1);
                }
                // if value is inexact, don't use it
                if (s.size() >= 3 && s.substr(s.size() - 3) == "...") {
                    return false;
                }
                mpf_class x_tmp {s};
                x = x_tmp;
            } else {
                std::size_t i = 0;
                for (; i < s.size(); ++i) {
                    if (!std::isspace(s[i])) {
                        break;
                    }
                }
                std::size_t num_start = i;
                for (; i < s.size(); ++i) {
                    if (!(s[i] == '-' || s[i] == '+' || std::isdigit(s[i]))) {
                        break;
                    }
                }
                std::size_t num_end = i;
                mpz_class num {s.substr(num_start, num_end - num_start)};
                for (; i < s.size(); ++i) {
                    if (!std::isspace(s[i])) {
                        break;
                    }
                }
                if (i == s.size()) {
                    // value is integer
                    x.get_num().swap(num);
                    x.get_den() = 1;
                    x.canonicalize();
                    return true;
                } else if (s[i] == '/') {
                    // fraction
                    mpz_class den{s.substr(i + 1)};
                    x.get_num().swap(num);
                    x.get_den().swap(den);
                    x.canonicalize();
                } else {
                    // mixed fraction
                    mpq_class rest{s.substr(i + 1)};
                    x = rest;
                    x += num;
                }
            }
        } catch(std::invalid_argument&) {
            return false;
        }
        return true;
    }
};

int main(int argc, char** argv) {
    auto gtkApp = Gtk::Application::create(argc, argv, "",
                      Gio::APPLICATION_NON_UNIQUE | Gio::APPLICATION_HANDLES_COMMAND_LINE);

    std::istream* prog_file = &std::cin;
    std::string prog_filename = "-";
    std::istream* input_file = nullptr;
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
        prog_file = new std::ifstream {prog_filename};
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
        input_file = new std::ifstream {input_filename};
        if (!*input_file) {
            std::cerr << "Failed to open file: " << input_filename << "\n";
            std::exit(1);
        }
    }

    std::wstring_convert <std::codecvt_utf8 <TChar>, TChar> utf8_convert;
    std::string prog_text;
    {
        char c;
        while (prog_file->get(c)) {
            prog_text += c;
        }
    }
    TState state(utf8_convert.from_bytes(prog_text));

    std::string input_text;
    if (input_file) {
        char c;
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
