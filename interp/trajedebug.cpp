/*
 * Simple interpreter for Trajedy.
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
#include <gtkmm.h>
#include "program_option.hpp"

bool debug_mode = false;
std::ostream dummy_ostream(nullptr);
std::ostream& debug() {
    if (debug_mode) {
        return std::cerr;
    } else {
        return dummy_ostream;
    }
}

/*
 * Basic pointer movement and raycasting helpers.
 */
template <class N>
struct v2
{
    N x, y;

    v2() = default;
    v2(const v2&) = default;
    v2(v2&&) = default;
    v2& operator=(const v2&) = default;
    v2(N a): v2(a, a) { }
    v2(N x, N y): x(x), y(y) { }

    template <class N2>
    explicit v2(const v2 <N2>& v): x(v.x), y(v.y) { }

    v2 operator+(const v2& v) const {
        v2 t {*this};
        t += v;
        return t;
    }
    v2& operator+=(const v2& v) {
        x += v.x, y += v.y;
        return *this;
    }
    v2 operator-(const v2& v) const {
        v2 t {*this};
        t -= v;
        return t;
    }
    v2& operator-=(const v2& v) {
        x -= v.x, y -= v.y;
        return *this;
    }
    v2 operator*(const v2& v) const {
        v2 t {*this};
        t *= v;
        return t;
    }
    v2& operator*=(const v2& v) {
        x *= v.x, y *= v.y;
        return *this;
    }
    v2 operator-() const {
        v2 t(*this);
        t = -t;
        return t;
    }
    v2& operator-() {
        x = -x, y = -y;
        return *this;
    }
    bool operator==(const v2& v) const {
        return x == v.x && y == v.y;
    }
    bool operator!=(const v2& v) const {
        return !(*this == v);
    }

    N norm2() const {
        return x*x + y*y;
    }

    void swap(v2& v) {
        std::swap(x, v.x);
        std::swap(y, v.y);
    }

    template <class N2>
    explicit operator v2 <N2> () const {
        return v2 <N2> (x, y);
    }
};

namespace std {
    template <class N>
    void swap(v2 <N>& u, v2 <N>& v) {
        u.swap(v);
    }
}

template <class N>
std::ostream& operator<<(std::ostream& out, const v2 <N>& v) {
    return out << '(' << v.x << ", " << v.y << ')';
}

/*
 * Trajedy pointer, which travels in the program area.
 * This class does not keep track of the pointer's I/O modes.
 */
struct TPointer
{
    /*
     * This is the current square. (The square at (0,0) is the one that
     * covers the points [0,1] × [0,1].) This information is important
     * when we cross a grid boundary.
     */
    v2 <long> current_square;

    /*
     * The exact location of the pointer.
     */
    v2 <mpq_class> position;

    /*
     * The direction of the pointer, as an (x,y) pair.
     * The x and y are always coprime.
     */
    v2 <mpz_class> direction;

    /*
     * Steps into the next square in the path.
     * This assumes that the pointer is not is_along_edge().
     */
    TPointer& operator++() {
        assert (!is_along_edge());
        assert (gcd(direction.x, direction.y) == 1);

        // Helper to move from somewhere on (or in) a square to an
        // adjacent square, according to the pointer direction.
        auto move = [&]() -> void {
            // If direction parallel to an axis, then move along it
            if (direction.x == 0) {
                // NB: we know direction.y is normalized to ±1
                current_square.y += sgn(direction.y);
                position.y = current_square.y;
                if (direction.y < 0) {
                    position.y += 1;
                }
            }
            else if (direction.y == 0) {
                current_square.x += sgn(direction.x);
                position.x = current_square.x;
                if (direction.x < 0) {
                    position.x += 1;
                }
            }
            // Otherwise, try both axes and pick the nearest intersection
            else {
                mpz_class x_target;
                if (direction.x < 0) {
                    mpz_cdiv_q(x_target.get_mpz_t(),
                               position.x.get_num_mpz_t(), position.x.get_den_mpz_t());
                    x_target -= 1;
                } else /* direction.x > 0 */ {
                    mpz_fdiv_q(x_target.get_mpz_t(),
                               position.x.get_num_mpz_t(), position.x.get_den_mpz_t());
                    x_target += 1;
                }
                v2 <mpq_class> x_travel;
                x_travel.x = x_target - position.x;
                x_travel.y = x_travel.x * direction.y / direction.x;

                mpz_class y_target;
                if (direction.y < 0) {
                    mpz_cdiv_q(y_target.get_mpz_t(),
                               position.y.get_num_mpz_t(), position.y.get_den_mpz_t());
                    y_target -= 1;
                } else /* direction.y > 0 */ {
                    mpz_fdiv_q(y_target.get_mpz_t(),
                               position.y.get_num_mpz_t(), position.y.get_den_mpz_t());
                    y_target += 1;
                }
                v2 <mpq_class> y_travel;
                y_travel.y = y_target - position.y;
                y_travel.x = y_travel.y * direction.x / direction.y;

                // Check travel along the x-axis.
                if ((direction.x > 0) ==
                    (x_travel.x < y_travel.x)) {
                    position += x_travel;
                } else {
                    position += y_travel;
                }

                // Enter the new square.
                current_square.x = mpz_class(position.x.get_num() / position.x.get_den()).get_si();
                if (position.x == current_square.x && direction.x < 0) {
                    current_square.x -= 1;
                }
                current_square.y = mpz_class(position.y.get_num() / position.y.get_den()).get_si();
                if (position.y == current_square.y && direction.y < 0) {
                    current_square.y -= 1;
                }
            }
        };

        // Are we on a boundary?
        bool x_edge {x_on_edge()}, y_edge {y_on_edge()};
        if (x_edge || y_edge) {
            v2 <long> dir_sign = {
                sgn(direction.x),
                sgn(direction.y)
            };
            // Is is a corner?
            if (x_edge && y_edge) {
                v2 <long> from_square = {
                    // NB: get_den() == 1, so get_num() is the position
                    position.x.get_num().get_si() - 1,
                    position.y.get_num().get_si() - 1
                };
                if (dir_sign.x < 0) {
                    from_square.x++;
                }
                if (dir_sign.y < 0) {
                    from_square.y++;
                }
                debug() << "corner squares: " << from_square << " ->? " << current_square << "\n";
                if (from_square == current_square) {
                    // Step into opposite square without moving
                    current_square += dir_sign;
                } else {
                    // Travel across current square
                    move();
                }
                return *this;
            }

            // Not corner. Proceed as above, but for only the edge axis.
            if (x_edge) {
                v2 <long> from_square = {
                    position.x.get_num().get_si() - 1,
                    current_square.y
                };
                if (dir_sign.x < 0) {
                    from_square.x++;
                }
                if (from_square == current_square) {
                    current_square.x += dir_sign.x;
                } else {
                    move();
                }
                return *this;
            }

            if (y_edge) {
                v2 <long> from_square = {
                    current_square.x,
                    position.y.get_num().get_si() - 1
                };
                if (dir_sign.y < 0) {
                    from_square.y++;
                }
                if (from_square == current_square) {
                    current_square.y += dir_sign.y;
                } else {
                    move();
                }
                return *this;
            }
        }

        // Not on any boundary; move to the next boundary.
        move();
        return *this;
    }
    TPointer operator++(int) const {
        TPointer p {*this};
        return ++p;
    }

    // NB: foo_on_edge is for edges perpendicular to—not parallel to—
    //     the foo axis.
    bool x_on_edge() const {
        return position.x.get_den() == 1;
    }
    bool y_on_edge() const {
        return position.y.get_den() == 1;
    }
    bool on_boundary() const {
        return x_on_edge() || y_on_edge();
    }
    bool on_edge() const {
        return x_on_edge() != y_on_edge();
    }
    bool on_corner() const {
        return x_on_edge() && y_on_edge();
    }
    bool is_along_edge() const {
        return (x_on_edge() && direction.x == 0) ||
               (y_on_edge() && direction.y == 0);
    }
};

/*
 * Trajedy program state, including pointer, mode and program area.
 */
using TChar = char; // TODO
struct TState {
    static const TChar
        newline = '\n',
        comma   = ',',
        period  = '.',
        mirror1 = '/',
        mirror2 = '\\',
        space   = ' ';

    // Grid of instructions.
    // squares always has at least one row; this makes things easier.
    std::vector <std::vector <TChar>> squares;

    // Instruction pointer.
    TPointer ptr;

    // Program mode.
    enum Mode {
        NormalMode,
        InputMode,
        OutputMode,
    };

    Mode mode;

    // Initialize program state from text.
    TState(const std::basic_string <TChar>& source):
        ptr { {-1, -1}, {0, 0}, {1, 1} },
        mode(NormalMode)
    {
        squares.resize(1);
        for (TChar c: source) {
            squares.back().push_back(c);
            if (c == newline) {
                squares.resize(squares.size() + 1);
            }
        }

        std::size_t longest_line = 0;
        for (const auto& row: squares) {
            longest_line = std::max(longest_line, row.size());
        }
        for (auto& row: squares) {
            row.resize(longest_line, space);
        }
    }

    TState() = delete;
    TState(const TState&) = default;

    std::size_t width() const {
        return squares.at(0).size();
    }
    std::size_t height() const {
        return squares.size();
    }

    // Helpers for changing pointer direction.

    // Find the nearest point on the given beacon square.
    v2 <mpq_class> nearest_point(v2 <long> beacon) const {
        // Travelling to the same square doesn't really make sense.
        assert (ptr.current_square != beacon);

        /*
         * If it is on a different row and column, the nearest point
         * is a corner. This test might be off if the pointer is also
         * on a corner, but it doesn't really matter in that case.
         */
        if (beacon.x != ptr.current_square.x && beacon.y != ptr.current_square.y) {
            v2 <mpq_class> target {beacon};
            if (beacon.x < ptr.current_square.x) {
                target.x += 1;
            }
            if (beacon.y < ptr.current_square.y) {
                target.y += 1;
            }
            return target;
        }

        /*
         * If it is on the same row or column, the nearest point is
         * on an edge (and may be on a corner); the line of travel
         * is parallel to an axis.
         */
        else if (beacon.x == ptr.current_square.x) {
            v2 <mpq_class> target {ptr.position.x, beacon.y};
            if (beacon.y < ptr.current_square.y) {
                target.y += 1;
            }
            return target;
        }
        else /* beacon.y == ptr.current_square.y */ {
            v2 <mpq_class> target {beacon.x, ptr.position.y};
            if (beacon.x < ptr.current_square.x) {
                target.x += 1;
            }
            return target;
        }
    }

    // Find the nearest beacon and turn towards it.
    // If there is no nearest beacon, do nothing.
    // Returns the target beacon location (or {-1, -1}).
    std::tuple <v2 <mpq_class>, v2 <long>> turn_beacon(TChar beacon_c) {
        // <squared distance, target point, target square>
        using entry = std::tuple <mpq_class, v2 <mpq_class>, v2 <long>>;
        std::vector <entry> targets;
        // TODO: precompute these
        for (std::size_t row = 0; row < height(); ++row) {
            for (std::size_t col = 0; col < width(); ++col) {
                v2 <long> target_square(col, row);
                if (ptr.current_square == target_square) {
                    continue;
                }
                if (squares[row][col] == beacon_c) {
                    v2 <mpq_class> this_target = nearest_point(target_square);
                    mpq_class this_dist = (this_target - ptr.position).norm2();
                    debug() << "Beacon: " << target_square << " at " << this_target
                            << ", dist " << this_dist << "\n";
                    targets.emplace_back(this_dist, this_target, target_square);
                }
            }
        }
        std::sort(targets.begin(), targets.end(),
                  [](const entry& e1, const entry& e2) {
                      return std::get<0>(e1) < std::get<0>(e2);
                  });
        if (targets.empty()) {
            // No beacons
            return std::make_tuple(v2 <mpq_class> {-1, -1}, v2 <long> {-1, -1});
        } else if (targets.size() > 1 &&
                   std::get<0>(targets[0]) == std::get<0>(targets[1])) {
            // No unique nearest beacon
            return std::make_tuple(v2 <mpq_class> {-1, -1}, v2 <long> {-1, -1});
        } else {
            // Calculate new direction
            const auto& target = std::get<1>(targets[0]);
            // If the target is directly adjacent, we need special handling.
            if (target == ptr.position) {
                ptr.direction = v2 <mpz_class> (std::get<2>(targets[0]) - ptr.current_square);
            } else {
                v2 <mpq_class> travel {target - ptr.position};
                ptr.direction.x = travel.x.get_num() * travel.y.get_den();
                ptr.direction.y = travel.y.get_num() * travel.x.get_den();
                mpz_class factor {gcd(ptr.direction.x, ptr.direction.y)};
                mpz_divexact(ptr.direction.x.get_mpz_t(),
                             ptr.direction.x.get_mpz_t(), factor.get_mpz_t());
                mpz_divexact(ptr.direction.y.get_mpz_t(),
                             ptr.direction.y.get_mpz_t(), factor.get_mpz_t());
            }
            return std::make_tuple(std::get<1>(targets[0]), std::get<2>(targets[0]));
        }
    }

    /*
     * Reflect pointer off a mirror. If its path would miss the mirror,
     * the pointer is left unchanged. Otherwise, it is moved to the
     * mirror's surface and its direction is updated.
     *
     * Returns true if the pointer was reflected.
     */
    bool reflect() {
        TChar mirror_c = current_square();
        // There must be a mirror here.
        assert (mirror_c == mirror1 || mirror_c == mirror2);

        int sign = mirror_c == mirror1? -1 : 1; // slope
        v2 <long> mirror_start = ptr.current_square;
        if (mirror_c == mirror1) {
            mirror_start.y += 1;
        }

        /*
         * Intersection point, if any:
         *
         * (x, y) + t*(dx, dy) = (mx, my) + (u, sign*u)
         * 0 <= u <= 1
         *
         * x + t*dx = mx + u  &&  y + t*dy = my + sign*u
         * u = x + t*dx - mx = sign*(y + t*dy - my)
         * t = (-x + sign*y + mx - sign*my) / (dx - sign*dy)  [dx != sign*dy]
         */
        if (ptr.direction.x == sign * ptr.direction.y) {
            // Parallel to mirror
            return false;
        }
        mpq_class t { (-ptr.position.x + sign * ptr.position.y +
                       mirror_start.x - sign * mirror_start.y) /
                      mpq_class(ptr.direction.x - sign * ptr.direction.y) };
        mpq_class u {ptr.position.x + t * ptr.direction.x - mirror_start.x};
        if (0 <= u && u <= 1) {
            // Move to the intersection point.
            debug() << "Reflect t=" << t << ", u=" << u << "\n";
            ptr.position.x = mirror_start.x + u;
            ptr.position.y = mirror_start.y + sign*u;

            // Reflect off the mirror.
            if (mirror_c == mirror1) {
                ptr.direction = v2 <mpz_class> {-ptr.direction.y, -ptr.direction.x};
            } else {
                ptr.direction = v2 <mpz_class> {ptr.direction.y, ptr.direction.x};
            }
            return true;
        } else {
            return false;
        }
    }

    // Access the current square's value. The pointer must be in range.
    TChar current_square() const {
        assert (pointer_in_range());
        return squares[ptr.current_square.y][ptr.current_square.x];
    }
    TChar& current_square() {
        assert (pointer_in_range());
        return squares[ptr.current_square.y][ptr.current_square.x];
    }
    // Note that the pointer is initialised to an out-of-range square (-1, -1).
    bool pointer_in_range() const {
        return ptr.current_square.x >= 0 && std::size_t(ptr.current_square.x) < width() &&
               ptr.current_square.y >= 0 && std::size_t(ptr.current_square.y) < height();
    }
};
const TChar
    TState::newline,
    TState::comma,
    TState::period,
    TState::mirror1,
    TState::mirror2,
    TState::space;

/*
 * Undo record for program state.
 */
struct TChange {
    TPointer old_ptr;
    TState::Mode old_mode;

    // Store overwritten character, if any.
    TChar old_char;

    // Currently unused, but kept updated.
    TChar output_char;

    // For mirrors, saves the point where the pointer reflected.
    bool was_mirror;
    v2 <mpq_class> mirror_hit;
};

/*
 * Debugging backend.
 */
struct DebugState {
    TState state;
    std::deque <TChange> undos;
    std::deque <TChar> input, output;

    // These are not needed by the debugger, but are useful for display
    std::deque <TChar> past_input, future_output;

    DebugState(TState state): state(state)
    { }

    // Advance one step of the program. Does not check pointer validity.
    void step() {
        TChange change = { state.ptr, state.mode, 0, 0, false, {0, 0} };

        if (state.pointer_in_range()) {
            TChar op = state.current_square();
            if (state.mode == TState::NormalMode) {
                if (op == TState::space) {
                    // no-op
                } else if (op == TState::mirror1 || op == TState::mirror2) {
                    if (state.reflect()) {
                        change.was_mirror = true;
                        change.mirror_hit = state.ptr.position;
                    }
                } else if (op == TState::comma) {
                    state.mode = TState::InputMode;
                } else if (op == TState::period) {
                    state.mode = TState::OutputMode;
                } else {
                    state.turn_beacon(op);
                }
            }
            else if (state.mode == TState::InputMode) {
                change.old_char = state.current_square();
                if (input.empty()) {
                    state.current_square() = TChar(0);
                } else {
                    debug() << "input buffer: ";
                    for (TChar c: input) {
                        debug() << "\\x" << std::hex << unsigned(c);
                    }
                    debug() << "\n";
                    state.current_square() = input.front();
                    past_input.push_back(input.front());
                    input.pop_front();
                }
                debug() << "input: \\x" << std::hex << unsigned(state.current_square()) << "\n";
                state.mode = TState::NormalMode;
            }
            else if (state.mode == TState::OutputMode) {
                change.output_char = state.current_square();
                output.push_back(change.output_char);
                if (!future_output.empty()) {
                    future_output.pop_front();
                }
                state.mode = TState::NormalMode;
            }
        }

        ++state.ptr;
        undos.push_back(change);
    }

    // Go back one step in the program.
    void unstep() {
        if (!undos.empty()) {
            TChange& change = undos.back();
            state.ptr = change.old_ptr;
            if (change.old_mode == TState::InputMode) {
                if (state.current_square() == TChar(0)) {
                    // was EOF
                    state.current_square() = change.old_char;
                } else {
                    assert (!past_input.empty());
                    input.push_front(state.current_square());
                    state.current_square() = change.old_char;
                    past_input.pop_back();
                }
            }
            if (change.old_mode == TState::OutputMode) {
                assert (!output.empty() && change.output_char == output.back());
                future_output.push_front(change.output_char);
                output.pop_back();
            }
            state.mode = change.old_mode;
            undos.pop_back();
        }
    }

    // Change user input.
    const std::deque <TChar>& get_input() const {
        return input;
    }
    void set_input(std::deque <TChar>&& new_input) {
        input = new_input;
        future_output.clear();
    }
};

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
            // Pointer is invalid
            cairo->set_source_rgb(0.8, 0.2, 0.0);
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

    Gtk::Label control_bar_label;
    Gtk::Button reset;
    Gtk::Button undo;
    Gtk::ToggleButton stop;
    Gtk::Button step;
    Gtk::ToggleButton run;
    Gtk::HSeparator control_bar_hsep;

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

    std::function <void(void)> update_step_counter;

    MainWindow(TState state):
        main_panels {Gtk::ORIENTATION_HORIZONTAL},
        control_panels {Gtk::ORIENTATION_VERTICAL, 0},
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
                        background_runner = Glib::signal_idle().connect(step_program);
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
                        background_runner = Glib::signal_idle().connect(step_program);
                    } else {
                        background_runner.disconnect();
                    }
                }
            });
        control_bar.pack_start(run);
        control_panels.pack_start(control_bar_hsep);

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

        ('d', "debug", "enable verbose debug mode",
         []() { debug_mode = true; })

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
