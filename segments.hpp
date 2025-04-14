#include <cstdint>
#include <bitset>
#include <string_view>

extern "C" std::uint16_t GetCS();
extern "C" std::uint16_t GetDS();
extern "C" std::uint16_t GetSS();
extern "C" std::uint16_t GetES();
extern "C" std::uint16_t GetFS();
extern "C" std::uint16_t GetGS();

class Segment {
public:
    enum class Privilege {
        KERNEL = 0b00,
        DRIVER_KERNEL = 0b01,
        DRIVER_USER = 0b10,
        USER = 0b11,
    };
protected:
    std::uint16_t m_segment;
    Privilege m_privilege;
public:
    constexpr Segment(std::uint16_t segment) noexcept : m_segment(segment) {
        m_privilege = static_cast<Privilege>(m_segment & 0b11);
    }

    constexpr std::uint16_t GetSegmentAddress() const noexcept { return m_segment >> 3; }
    constexpr std::string_view GetSegmentAddressBits() const noexcept {
        return std::bitset<8>(GetSegmentAddress()).to_string();
    }

    constexpr bool IsLDT() const noexcept { return !IsGDT(); }
    constexpr bool IsGDT() const noexcept { return !(m_segment & 0b100); }

    constexpr std::string_view GetPrivilege() const noexcept {
        Privilege p = static_cast<Privilege>(m_segment & 0b11);
        switch(p) {
            case Privilege::KERNEL:
                return "KERNEL";
            case Privilege::DRIVER_KERNEL:
                return "DRIVER_KERNEL";
            case Privilege::DRIVER_USER:
                return "DRIVER_USER";
            case Privilege::USER:
                return "USER";
        }
        return "?";
    }

    constexpr std::bitset<16> AsBitset() const noexcept { return m_segment; }
    constexpr std::string_view GetBits() const noexcept { return AsBitset().to_string(); }
    constexpr std::string_view GetBitsPretty() const noexcept {
        std::string_view bs = GetBits();
        return std::string(bs.substr(0, 13)) + " " + std::string(bs.substr(13, 1)) + " " + std::string(bs.substr(14, 2));
    }
};
