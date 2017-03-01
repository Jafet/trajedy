/*
 * Simple interpreter for Trajedy.
 */

#include <algorithm>
#include <cassert>
#include <codecvt>
#include <cstdlib>
#include <fstream>
#include <gmpxx.h>
#include <iostream>
#include <iterator>
#include <locale>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "program_option.hpp"
#include "v2.hpp"
#include "interp.hpp"

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
 * Note: we currently do all I/O (not just reading the program text) in UTF-8,
 * ignoring the user's locale. Most users probably have a UTF-8 locale anyway.
 */
int main(int argc, char** argv) {
    auto dump_pointer = [](const TPointer& ptr) {
        debug() << ptr.position << "[in " << ptr.current_square << "] + "
                << ptr.direction << "\n";
    };

    std::istream* prog_file = &std::cin;
    std::string prog_filename = "-";
    std::istream* input_file = &std::cin;
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

    if (prog_filename != "-") {
        prog_file = new std::ifstream {prog_filename};
        if (!*prog_file) {
            std::cerr << "Failed to open file: " << prog_filename << "\n";
            std::exit(1);
        }
    }

    if (input_filename != "-") {
        input_file = new std::ifstream {input_filename};
        if (!*input_file) {
            std::cerr << "Failed to open file: " << input_filename << "\n";
            std::exit(1);
        }
    }

    /*
     * XXX: GCC 6 may have a bug where converting cin.rdbuf() does not work.
     * We work around it by opening /dev/stdin instead.
     */
    if (input_filename == "-") {
        input_file = new std::ifstream {"/dev/stdin"};
    }
    std::wstring_convert <std::codecvt_utf8 <TChar>, TChar> utf8_convert;
    std::wbuffer_convert <std::codecvt_utf8 <TChar>, TChar> buf_w {input_file->rdbuf()};
    std::basic_istream <TChar> input_w {&buf_w};

    std::string prog_bytes
        { std::istream_iterator <char> {*prog_file >> std::noskipws}, {} };
    std::basic_string <TChar> prog_chars = utf8_convert.from_bytes(prog_bytes);
    TState state(prog_chars);

    debug() << "Program text (newlines denoted by NL):\n";
    for (auto const& row: state.squares) {
        for (auto c: row) {
            if (c == TState::newline) {
                debug() << "NL ";
            } else {
                debug() << '[' << utf8_convert.to_bytes(c) << ']';
            }
        }
        debug() << '\n';
    }

    dump_pointer(state.ptr);
    // FIXME: should share code with DebugState::step
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
            } else if (op == TState::question_mark) {
                state.mode = TState::SpecialCharMode;
            } else {
                std::string op_utf8 = utf8_convert.to_bytes(op);
                debug() << "Beacon: [" << op_utf8 << "]\n";
                state.turn_beacon(op);
            }
        }
        else if (state.mode == TState::InputMode) {
            TChar c;
            if (!input_w.get(c)) {
                debug() << "Input: end-of-input marker\n";
                c = TState::EOI;
            } else {
                std::string c_utf8 = utf8_convert.to_bytes(c);
                debug() << "Input: [" << c_utf8 << "]\n";
            }
            state.current_square() = c;
            state.mode = TState::NormalMode;
        }
        else if (state.mode == TState::OutputMode) {
            TChar c = state.current_square();
            if (c != TState::EOI) {
                std::string c_utf8 = utf8_convert.to_bytes(c);
                std::cout << c_utf8;
                debug() << "Output: [" << c_utf8 << "]\n";
            } else {
                debug() << "Output: end-of-input marker\n";
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
}
