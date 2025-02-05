#pragma once

#include <string>
#include <memory>
#include <concepts>

namespace Common {
	template <typename Type>
	constexpr bool is_cstring = std::is_same_v<std::remove_cvref_t<Type>, char*>
		|| std::is_same_v<std::remove_cvref_t<Type>, wchar_t*>;

	template <typename Type>
	constexpr bool is_string = std::is_same_v<std::remove_cvref_t<Type>, std::string>
		|| std::is_same_v<std::remove_cvref_t<Type>, std::wstring>;

	template <typename Type>
	class UString_Base {
	public:
		using value_type = Type;

	protected:
		std::unique_ptr<value_type> m_data;
		std::size_t m_length;
	public:
		UStringBase() {

		}
	};
}