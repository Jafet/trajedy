/*
 * Trajedy interpreter backend (implementation).
 */

#include "interp.hpp"

#include <algorithm>
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

/*
 * Steps into the next square in the path.
 * This assumes that the pointer is not is_along_edge().
 */
TPointer& TPointer::operator++() {
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

/*
 * Trajedy program state, including pointer, mode and program area.
 */

// Find the nearest point on the given beacon square.
v2 <mpq_class> TState::nearest_point(v2 <long> beacon) const {
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
std::tuple <v2 <mpq_class>, v2 <long>> TState::turn_beacon(TChar beacon_c) {
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
bool TState::reflect() {
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

// Static members
const TChar
    TState::newline,
    TState::comma,
    TState::period,
    TState::mirror1,
    TState::mirror2,
    TState::space,
    TState::EOI,
    TState::EOI_target;

/*
 * Debugging backend.
 */

// Advance one step of the program. Does not check pointer validity.
// One step consists of running the current instruction (if there is one),
// then moving the pointer (if possible).
void DebugState::step() {
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
            } else if (op == TState::question_mark) {
                state.mode = TState::SpecialCharMode;
            } else {
                state.turn_beacon(op);
            }
        }
        else if (state.mode == TState::InputMode) {
            change.old_char = state.current_square();
            if (input.empty()) {
                state.current_square() = TState::EOI;
            } else {
                state.current_square() = input.front();
                past_input.push_back(input.front());
                input.pop_front();
            }
            state.mode = TState::NormalMode;
        }
        else if (state.mode == TState::OutputMode) {
            TChar c = state.current_square();
            if (c != TState::EOI) {
                change.output_char = c;
                output.push_back(change.output_char);
                if (!future_output.empty()) {
                    future_output.pop_front();
                }
            }
            state.mode = TState::NormalMode;
        }
        else if (state.mode == TState::SpecialCharMode) {
            TChar c = state.current_square();
            if (c == TState::comma) {
                state.turn_beacon(TState::comma_target);
            } else if (c == TState::period) {
                state.turn_beacon(TState::period_target);
            } else if (c == TState::question_mark) {
                state.turn_beacon(TState::question_target);
            } else if (c == TState::EOI) {
                state.turn_beacon(TState::EOI_target);
            }
            state.mode = TState::NormalMode;
        }
    }

    if (!state.ptr.is_along_edge()) {
        ++state.ptr;
    }
    undos.push_back(change);
}

// Go back one step in the program.
void DebugState::unstep() {
    if (!undos.empty()) {
        TChange& change = undos.back();
        state.ptr = change.old_ptr;
        if (change.old_mode == TState::InputMode) {
            if (state.current_square() == TState::EOI) {
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
