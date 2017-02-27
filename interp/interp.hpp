/*
 * Trajedy interpreter backend.
 *
 * Some of the complicated functions are defined separately.
 */

#pragma once
#include <cassert>
#include <deque>
#include <gmpxx.h>
#include <string>
#include <tuple>
#include <vector>
#include "v2.hpp"

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
    TPointer& operator++();
    TPointer operator++(int) {
        TPointer p {*this};
        operator++();
        return p;
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
    v2 <mpq_class> nearest_point(v2 <long> beacon) const;

    // Find the nearest beacon and turn towards it.
    // If there is no nearest beacon, do nothing.
    // Returns the target beacon location (or {-1, -1}).
    std::tuple <v2 <mpq_class>, v2 <long>> turn_beacon(TChar beacon_c);

    /*
     * Reflect pointer off a mirror. If its path would miss the mirror,
     * the pointer is left unchanged. Otherwise, it is moved to the
     * mirror's surface and its direction is updated.
     *
     * Returns true if the pointer was reflected.
     */
    bool reflect();

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
    // One step consists of running the current instruction (if there is one),
    // then moving the pointer (if possible).
    void step();

    // Go back one step in the program.
    void unstep();

    // Change user input.
    const std::deque <TChar>& get_input() const {
        return input;
    }
    void set_input(std::deque <TChar>&& new_input) {
        input = new_input;
        future_output.clear();
    }
};
