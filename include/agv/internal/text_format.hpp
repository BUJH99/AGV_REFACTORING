#pragma once

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

namespace agv::internal::text {

struct PrintfSpec final {
    char conversion{'\0'};
    bool left_align{false};
    bool has_width{false};
    int width{0};
    bool has_precision{false};
    int precision{0};
};

inline void append_with_width(std::string& out, std::string_view value, const PrintfSpec& spec) {
    std::string_view text = value;
    if (spec.has_precision && spec.conversion == 's' &&
        static_cast<std::size_t>(spec.precision) < text.size()) {
        text = text.substr(0, static_cast<std::size_t>(spec.precision));
    }

    if (!spec.has_width || spec.width <= static_cast<int>(text.size())) {
        out.append(text);
        return;
    }

    const std::size_t padding = static_cast<std::size_t>(spec.width - static_cast<int>(text.size()));
    if (spec.left_align) {
        out.append(text);
        out.append(padding, ' ');
        return;
    }

    out.append(padding, ' ');
    out.append(text);
}

inline std::string to_string_value(std::string_view value) {
    return std::string(value);
}

inline std::string to_string_value(const std::string& value) {
    return value;
}

inline std::string to_string_value(const char* value) {
    return value ? std::string(value) : std::string("(null)");
}

inline std::string to_string_value(char* value) {
    return value ? std::string(value) : std::string("(null)");
}

template <std::size_t N>
inline std::string to_string_value(const char (&value)[N]) {
    return std::string(value);
}

template <typename T>
inline std::string to_string_value(const T& value) {
    std::ostringstream stream;
    stream << value;
    return stream.str();
}

template <typename T>
inline long long to_signed_value(const T& value) {
    using Decayed = std::remove_cvref_t<T>;
    if constexpr (std::is_enum_v<Decayed>) {
        return static_cast<long long>(static_cast<std::underlying_type_t<Decayed>>(value));
    } else if constexpr (std::is_arithmetic_v<Decayed>) {
        return static_cast<long long>(value);
    } else {
        throw std::runtime_error("format conversion requires a signed numeric value");
    }
}

template <typename T>
inline unsigned long long to_unsigned_value(const T& value) {
    using Decayed = std::remove_cvref_t<T>;
    if constexpr (std::is_enum_v<Decayed>) {
        using Underlying = std::underlying_type_t<Decayed>;
        return static_cast<unsigned long long>(static_cast<std::make_unsigned_t<Underlying>>(value));
    } else if constexpr (std::is_arithmetic_v<Decayed>) {
        return static_cast<unsigned long long>(value);
    } else {
        throw std::runtime_error("format conversion requires an unsigned numeric value");
    }
}

template <typename T>
inline double to_double_value(const T& value) {
    using Decayed = std::remove_cvref_t<T>;
    if constexpr (std::is_arithmetic_v<Decayed>) {
        return static_cast<double>(value);
    } else {
        throw std::runtime_error("format conversion requires a floating-point value");
    }
}

template <typename T>
inline int to_int_value(const T& value) {
    using Decayed = std::remove_cvref_t<T>;
    if constexpr (std::is_enum_v<Decayed>) {
        return static_cast<int>(static_cast<std::underlying_type_t<Decayed>>(value));
    } else if constexpr (std::is_arithmetic_v<Decayed>) {
        return static_cast<int>(value);
    } else {
        throw std::runtime_error("format conversion requires an integer value");
    }
}

template <typename... Args>
class PrintfFormatter final {
public:
    explicit PrintfFormatter(Args&&... args)
        : args_(std::forward<Args>(args)...) {}

    std::string format(std::string_view format_string) {
        std::string output;
        output.reserve(format_string.size() + 64);

        for (std::size_t i = 0; i < format_string.size(); ++i) {
            const char ch = format_string[i];
            if (ch != '%') {
                output.push_back(ch);
                continue;
            }

            if (i + 1 < format_string.size() && format_string[i + 1] == '%') {
                output.push_back('%');
                ++i;
                continue;
            }

            PrintfSpec spec{};
            i = parse_spec(format_string, i + 1, spec);
            append_next_argument(output, spec);
        }

        return output;
    }

private:
    std::tuple<std::decay_t<Args>...> args_;
    std::size_t next_arg_index_{0};

    std::size_t parse_spec(std::string_view format_string, std::size_t index, PrintfSpec& spec) {
        while (index < format_string.size()) {
            if (format_string[index] == '-') {
                spec.left_align = true;
                ++index;
                continue;
            }
            break;
        }

        if (index < format_string.size() && format_string[index] == '*') {
            spec.has_width = true;
            spec.width = take_next_int_argument();
            if (spec.width < 0) {
                spec.left_align = true;
                spec.width = -spec.width;
            }
            ++index;
        } else {
            while (index < format_string.size() && std::isdigit(static_cast<unsigned char>(format_string[index]))) {
                spec.has_width = true;
                spec.width = (spec.width * 10) + (format_string[index] - '0');
                ++index;
            }
        }

        if (index < format_string.size() && format_string[index] == '.') {
            ++index;
            spec.has_precision = true;
            if (index < format_string.size() && format_string[index] == '*') {
                spec.precision = std::max(0, take_next_int_argument());
                ++index;
            } else {
                while (index < format_string.size() && std::isdigit(static_cast<unsigned char>(format_string[index]))) {
                    spec.precision = (spec.precision * 10) + (format_string[index] - '0');
                    ++index;
                }
            }
        }

        while (index < format_string.size()) {
            const char modifier = format_string[index];
            if (modifier == 'l' || modifier == 'h' || modifier == 'z') {
                ++index;
                continue;
            }
            break;
        }

        if (index >= format_string.size()) {
            throw std::runtime_error("unterminated printf-like format string");
        }

        spec.conversion = format_string[index];
        return index;
    }

    int take_next_int_argument() {
        int value = 0;
        with_argument(next_arg_index_++, [&value](const auto& argument) {
            value = to_int_value(argument);
        });
        return value;
    }

    void append_next_argument(std::string& output, const PrintfSpec& spec) {
        with_argument(next_arg_index_++, [&output, &spec](const auto& argument) {
            append_argument(output, spec, argument);
        });
    }

    template <typename Fn, std::size_t Index = 0>
    void with_argument(std::size_t argument_index, Fn&& fn) {
        using FnType = std::remove_reference_t<Fn>;
        with_argument_impl<FnType, Index>(argument_index, fn);
    }

    template <typename Fn, std::size_t Index = 0>
    void with_argument_impl(std::size_t argument_index, Fn& fn) {
        if constexpr (Index < sizeof...(Args)) {
            if (argument_index == Index) {
                fn(std::get<Index>(args_));
                return;
            }
            with_argument_impl<Fn, Index + 1>(argument_index, fn);
            return;
        }

        throw std::runtime_error("printf-like formatter received too few arguments");
    }

    template <typename T>
    static void append_argument(std::string& output, const PrintfSpec& spec, const T& argument) {
        switch (spec.conversion) {
        case 's':
            append_with_width(output, to_string_value(argument), spec);
            return;
        case 'c': {
            const char ch = static_cast<char>(to_int_value(argument));
            const std::string single_char(1, ch);
            append_with_width(output, single_char, spec);
            return;
        }
        case 'd':
        case 'i': {
            std::ostringstream stream;
            if (spec.has_width) {
                stream << std::setw(spec.width);
                if (spec.left_align) {
                    stream << std::left;
                }
            }
            stream << to_signed_value(argument);
            output += stream.str();
            return;
        }
        case 'u': {
            std::ostringstream stream;
            if (spec.has_width) {
                stream << std::setw(spec.width);
                if (spec.left_align) {
                    stream << std::left;
                }
            }
            stream << to_unsigned_value(argument);
            output += stream.str();
            return;
        }
        case 'f': {
            std::ostringstream stream;
            if (spec.has_width) {
                stream << std::setw(spec.width);
                if (spec.left_align) {
                    stream << std::left;
                }
            }
            stream << std::fixed << std::setprecision(spec.has_precision ? spec.precision : 6);
            stream << to_double_value(argument);
            output += stream.str();
            return;
        }
        default:
            throw std::runtime_error("unsupported printf-like conversion");
        }
    }
};

template <typename... Args>
inline std::string printf_like(std::string_view format_string, Args&&... args) {
    return PrintfFormatter<Args...>(std::forward<Args>(args)...).format(format_string);
}

template <typename... Args>
inline void console_print(std::string_view format_string, Args&&... args) {
    std::cout << printf_like(format_string, std::forward<Args>(args)...);
    std::cout.flush();
}

inline void console_write(std::string_view text) {
    std::cout << text;
    std::cout.flush();
}

}  // namespace agv::internal::text
