/*
 * Simple interpreter for Trajedy.
 */

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <fstream>
#include <gmpxx.h>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>
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
    v2 <ssize_t> current_square;

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
            v2 <ssize_t> dir_sign = {
                sgn(direction.x),
                sgn(direction.y)
            };
            // Is is a corner?
            if (x_edge && y_edge) {
                v2 <ssize_t> from_square = {
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
                v2 <ssize_t> from_square = {
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
                v2 <ssize_t> from_square = {
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
    enum {
        NormalMode,
        InputMode,
        OutputMode,
    } mode;

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

        size_t longest_line = 0;
        for (const auto& row: squares) {
            longest_line = std::max(longest_line, row.size());
        }
        for (auto& row: squares) {
            row.resize(longest_line, space);
        }
    }

    TState() = delete;
    TState(const TState&) = default;

    size_t width() const {
        return squares.at(0).size();
    }
    size_t height() const {
        return squares.size();
    }

    // Helpers for changing pointer direction.

    // Find the nearest point on the given beacon square.
    v2 <mpq_class> nearest_point(v2 <ssize_t> beacon) const {
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
    void turn_beacon(TChar beacon_c) {
        // <squared distance, target point, target square>
        using entry = std::tuple <mpq_class, v2 <mpq_class>, v2 <ssize_t>>;
        std::vector <entry> targets;
        // TODO: precompute these
        for (size_t row = 0; row < height(); ++row) {
            for (size_t col = 0; col < width(); ++col) {
                v2 <ssize_t> target_square(col, row);
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
        } else if (targets.size() > 1 &&
                   std::get<0>(targets[0]) == std::get<0>(targets[1])) {
            // No unique nearest beacon
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
        v2 <ssize_t> mirror_start = ptr.current_square;
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
        return ptr.current_square.x >= 0 && ptr.current_square.x < width() &&
               ptr.current_square.y >= 0 && ptr.current_square.y < height();
    }
};
const TChar
    TState::newline,
    TState::comma,
    TState::period,
    TState::mirror1,
    TState::mirror2,
    TState::space;

int main(int argc, char** argv) {
    auto dump_pointer = [](const TPointer& ptr) {
        debug() << ptr.position << "[in " << ptr.current_square << "] + "
                << ptr.direction << "\n";
    };

    std::basic_istream <TChar>* prog_file = &std::cin;
    std::string prog_filename = "-";
    std::basic_istream <TChar>* input_file = &std::cin;
    std::string input_filename = "-";
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

    if (prog_filename != std::string("-")) {
        prog_file = new std::basic_ifstream <TChar> {prog_filename};
        if (!*prog_file) {
            std::cerr << "Failed to open file: " << prog_filename << "\n";
            std::exit(1);
        }
    }

    if (input_filename != std::string("-")) {
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

    debug() << "Program text (newlines denoted by NL):\n";
    for (auto const& row: state.squares) {
        for (auto c: row) {
            if (c == TState::newline) {
                debug() << "NL ";
            } else {
                debug() << '[' << c << ']';
            }
        }
        debug() << '\n';
    }

    dump_pointer(state.ptr);
    for (;;) {
        if (state.ptr.is_along_edge()) {
            debug() << "Pointer along edge, aborting.\n";
            dump_pointer(state.ptr);
            std::exit(1);
        }
        ++state.ptr;
        dump_pointer(state.ptr);
        if (!state.pointer_in_range()) {
            debug() << "Exited normally.\n";
            break;
        }

        TChar op = state.current_square();
        if (state.mode == TState::NormalMode) {
            if (op == TState::space) {
                // no-op
            } else if (op == TState::mirror1 || op == TState::mirror2) {
                state.reflect();
            } else if (op == TState::comma) {
                state.mode = TState::InputMode;
            } else if (op == TState::period) {
                state.mode = TState::OutputMode;
            } else {
                state.turn_beacon(op);
            }
        }
        else if (state.mode == TState::InputMode) {
            TChar c;
            if (!input_file->get(c)) {
                c = 0;
            }
            state.current_square() = c;
            debug() << "Input: [" << c << "]\n";
            state.mode = TState::NormalMode;
        }
        else if (state.mode == TState::OutputMode) {
            std::cout << state.current_square();
            debug() << "Output: [" << state.current_square() << "]\n";
            state.mode = TState::NormalMode;
        }
    }
}
