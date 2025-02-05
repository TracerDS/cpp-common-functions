#pragma once

#include <string>
#include <vector>
#include <cstdlib>

#include <string>
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <cstddef>

namespace Common::UTF8 {
    constexpr std::size_t strlen(const char* str) noexcept {
        std::size_t len = 0;
        while (str[len++]);
        return len;
    }
    constexpr std::size_t wcslen(const wchar_t* str) noexcept {
        std::size_t len = 0;
        while (str[len++]);
        return len;
    }

    constexpr std::size_t mbstowcs(wchar_t* dest, const char* src, std::size_t max) noexcept {
        constexpr std::size_t ERR = static_cast<std::size_t>(-1);
        if (!src) {
            return ERR; // If src is null, return 0 as per standard
        }

        if (!dest) {
            // Count the number of wide characters without writing to dest
            std::size_t count = 0;
            const auto* current = reinterpret_cast<const std::uint8_t*>(src);

            while (*current) {
                auto c = *current;

                // Determine the number of bytes in the current multibyte sequence
                int numBytes = 1;
                if ((c & 0xE0) == 0xC0) numBytes = 2;
                else if ((c & 0xF0) == 0xE0) numBytes = 3;
                else if ((c & 0xF8) == 0xF0) numBytes = 4;

                if (numBytes == 1 || numBytes > 4) {
                    return ERR; // Invalid multibyte sequence
                }

                current += numBytes;
                ++count;
            }

            return count;
        }

        // Perform actual conversion
        std::size_t converted = 0;
        const char* current = src;

        while (*current && converted < max) {
            auto c = static_cast<std::uint8_t>(*current);

            // Determine the number of bytes in the current multibyte sequence
            int numBytes = 1;
            wchar_t wc = 0;

            if ((c & 0x80) == 0) {
                wc = c; // ASCII character
            } else if ((c & 0xE0) == 0xC0) {
                if (current + 1 >= src + max)
                    return ERR; // Insufficient input
                wc = ((c & 0x1F) << 6) | (current[1] & 0x3F);
                numBytes = 2;
            } else if ((c & 0xF0) == 0xE0) {
                if (current + 2 >= src + max)
                    return ERR; // Insufficient input
                wc = ((c & 0x0F) << 12) | ((current[1] & 0x3F) << 6) | (current[2] & 0x3F);
                numBytes = 3;
            } else if ((c & 0xF8) == 0xF0) {
                if (current + 3 >= src + max)
                    return ERR; // Insufficient input
                wc = ((c & 0x07) << 18) | ((current[1] & 0x3F) << 12) |
                    ((current[2] & 0x3F) << 6) | (current[3] & 0x3F);
                numBytes = 4;
            } else {
                return ERR; // Invalid multibyte sequence
            }

            if (converted + 1 >= max)
                return ERR; // Not enough space in dest
                
            dest[converted++] = wc;
            current += numBytes;
        }

        if (converted < max) {
            dest[converted] = L'\0'; // Null-terminate the wide character string
        }

        return converted;
    }

    constexpr std::size_t wcstombs(char* dest, const wchar_t* src, std::size_t max) noexcept {
        constexpr std::size_t ERR = static_cast<std::size_t>(-1);
        if (!src) {
            return ERR; // If src is null, return 1
        }

        if (!dest) {
            // Count the number of bytes required for the multibyte representation
            std::size_t count = 0;
            const wchar_t* current = src;

            while (*current != L'\0') {
                wchar_t wc = *current;

                if (wc <= 0x7F) {
                    count += 1; // ASCII character
                } else if (wc <= 0x7FF) {
                    count += 2; // 2-byte sequence
                } else if (wc <= 0xFFFF) {
                    count += 3; // 3-byte sequence
                } else if (wc <= 0x10FFFF) {
                    count += 4; // 4-byte sequence
                } else {
                    return ERR; // Invalid wide character
                }

                ++current;
            }

            return count;
        }

        // Perform actual conversion
        std::size_t converted = 0;
        const wchar_t* current = src;

        while (*current != L'\0' && converted < max) {
            wchar_t wc = *current;

            if (wc <= 0x7F) {
                if (converted + 1 > max) {
                    return ERR; // Not enough space in dest
                }
                dest[converted++] = static_cast<char>(wc);
            } else if (wc <= 0x7FF) {
                if (converted + 2 > max) {
                    return ERR; // Not enough space in dest
                }
                dest[converted++] = static_cast<char>(0xC0 | (wc >> 6));
                dest[converted++] = static_cast<char>(0x80 | (wc & 0x3F));
            } else if (wc <= 0xFFFF) {
                if (converted + 3 > max) {
                    return ERR; // Not enough space in dest
                }
                dest[converted++] = static_cast<char>(0xE0 | (wc >> 12));
                dest[converted++] = static_cast<char>(0x80 | ((wc >> 6) & 0x3F));
                dest[converted++] = static_cast<char>(0x80 | (wc & 0x3F));
            } else if (wc <= 0x10FFFF) {
                if (converted + 4 > max) {
                    return ERR; // Not enough space in dest
                }
                dest[converted++] = static_cast<char>(0xF0 | (wc >> 18));
                dest[converted++] = static_cast<char>(0x80 | ((wc >> 12) & 0x3F));
                dest[converted++] = static_cast<char>(0x80 | ((wc >> 6) & 0x3F));
                dest[converted++] = static_cast<char>(0x80 | (wc & 0x3F));
            } else {
                return ERR; // Invalid wide character
            }

            ++current;
        }

        if (converted < max) {
            dest[converted] = '\0'; // Null-terminate the multibyte string
        }

        return converted;
    }

    constexpr std::wstring fromUTF8(const std::wstring& utf16) noexcept { return utf16; }
    constexpr std::wstring fromUTF8(const std::string& utf8String) noexcept {
        if (utf8String.empty()) {
            return std::wstring{};
        }

        std::size_t wideCharCount = mbstowcs(nullptr, utf8String.c_str(), 0);
        if (wideCharCount == -1) {
            return std::wstring{};
        }

        std::vector<wchar_t> utf16Buffer(wideCharCount + 1);

        std::size_t result = mbstowcs(utf16Buffer.data(), utf8String.c_str(), utf8String.size());
        if (result == -1) {
            return std::wstring{};
        }

        return std::wstring(utf16Buffer.begin(), utf16Buffer.end() - 1);
    }

    constexpr std::string toUTF8(const std::string& utf8) noexcept { return utf8; }
    constexpr std::string toUTF8(const std::wstring& utf16String) noexcept {
        if (utf16String.empty()) {
            return std::string{};
        }

        std::size_t multiByteCount = wcstombs(nullptr, utf16String.c_str(), 0);
        if (multiByteCount == -1) {
            return std::string{};
        }

        std::vector<char> utf8Buffer(multiByteCount + 1);

        std::size_t result = wcstombs(utf8Buffer.data(), utf16String.c_str(), utf16String.size());
        if (result == -1) {
            return std::string{};
        }

        return std::string(utf8Buffer.begin(), utf8Buffer.end() - 1);
    }
}