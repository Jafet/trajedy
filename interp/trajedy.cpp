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
