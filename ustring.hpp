#pragma once

#include <string>
#include <memory>
#include <concepts>
#include <utf8.hpp>

namespace Common {
	template <typename Type>
	using remove_cvref_pointer = std::remove_cvref<std::remove_pointer_t<Type>>;

	template <typename Type>
	using remove_cvref_pointer_t = typename remove_cvref_pointer<Type>::type;

	template <typename Type>
	constexpr bool is_cstring = std::is_same_v<std::remove_cvref_t<Type>, char>
		|| std::is_same_v<std::remove_cvref_t<Type>, wchar_t>;

	template <typename Type>
	constexpr bool is_char = is_cstring<std::remove_pointer_t<Type>>;

	template <typename Type>
	concept char_concept = is_char<Type>;

	class UString : public std::string {
		using base = std::string;
		using base::basic_string;
	public:
		using value_type = base::value_type;
		using pointer = base::pointer;
		using const_pointer = base::const_pointer;
		using reference = base::reference;
		using const_reference = base::const_reference;
		using size_type = base::size_type;

	public:
		template <char_concept Type>
		constexpr UString(const Type* data) noexcept : base(UTF8::toUTF8(data)) {}

		constexpr UString(std::string data) noexcept : base(UTF8::toUTF8(data)) {}

		template <char_concept Type>
		constexpr UString(const Type* data, size_type count) noexcept : base(UTF8::toUTF8(data)) {}

		constexpr virtual ~UString() noexcept = default;
	};

	int main() {
		UString ustring = L"Hello, World!";

		std::cout << ustring;
	}
}