#pragma once

#include <string>
#include <vector>

namespace Common::UTF8  {
    constexpr std::wstring fromUTF8(const std::wstring& utf16) noexcept { return utf16; }
    std::wstring fromUTF8(const std::string& utf8String) noexcept {
        if (utf8String.empty())
            return {};

#ifdef _WIN32
        std::size_t wideCharCount;
        if (mbstowcs_s(
            &wideCharCount,
            nullptr,
            0,
            utf8String.c_str(),
            _TRUNCATE
        ) != 0 || wideCharCount == 0)
            return {};

        std::vector<wchar_t> utf16Buffer(wideCharCount);

        if (mbstowcs_s(
            &wideCharCount,
            utf16Buffer.data(),
            utf16Buffer.size(),
            utf8String.c_str(),
            _TRUNCATE
        ) != 0 || wideCharCount == 0)
            return {};
#else
        std::size_t wideCharCount = std::mbstowcs(
            nullptr,
            utf8String.c_str(),
            0
        );
        if (wideCharCount == -1)
            return {};

        std::vector<wchar_t> utf16Buffer(wideCharCount + 1);

        if (std::mbstowcs(
            utf16Buffer.data(),
            utf8String.c_str(),
            utf8String.size()
        ) == -1)
            return {};
#endif
        return std::wstring(utf16Buffer.begin(), utf16Buffer.end() - 1);
    }

    constexpr std::string toUTF8(const std::string& utf8) noexcept { return utf8; }
    std::string toUTF8(const std::wstring& utf16String) noexcept {
        if (utf16String.empty())
            return {};

#ifdef _WIN32
        std::size_t multiByteCount;
        if (wcstombs_s(
            &multiByteCount,
            nullptr,
            0,
            utf16String.c_str(),
            _TRUNCATE
        ) != 0 || multiByteCount == 0)
            return {};

        std::vector<char> utf8Buffer(multiByteCount);

        if (wcstombs_s(
            &multiByteCount,
            utf8Buffer.data(),
            utf8Buffer.size(),
            utf16String.c_str(),
            _TRUNCATE
        ) != 0 || multiByteCount == 0)
            return {};
#else
        std::size_t multiByteCount = std::wcstombs(
            nullptr,
            utf16String.c_str(),
            0
        );
        if (multiByteCount == -1)
            return {};

        std::vector<char> utf8Buffer(multiByteCount + 1);

        if (std::wcstombs(
            utf8Buffer.data(),
            utf16String.c_str(),
            utf16String.size()
        ) == -1)
            return {};
#endif
        return std::string(utf8Buffer.begin(), utf8Buffer.end() - 1);
    }
}
