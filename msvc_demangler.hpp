#pragma once

#include <string>
#include <string_view>
#include <cassert>

namespace Exports {
	constexpr std::string GetExportName(const std::string_view& signature) noexcept {
		assert(signature.size() > 5 && "Invalid signature size");
		assert(signature.starts_with('?') && "Signature must start with '?'");
		assert(signature.ends_with("Z") && "Signature must end with 'Z'");

		auto nameIndex = signature.find("@");
		assert(nameIndex != signature.npos && "Invalid signature");

		return std::string(signature.substr(1, nameIndex - 1));
	}

	constexpr std::string GetNamespaceName(const std::string_view& signature) noexcept {
		assert(signature.size() > 5 && "Invalid signature size");
		assert(signature.starts_with('?') && "Signature must start with '?'");
		assert(signature.ends_with("Z") && "Signature must end with 'Z'");

		auto nameIndex = signature.find("@");
		assert(nameIndex != signature.npos && "Invalid signature");

		auto namespaceIndex = signature.find("@@");
		assert(namespaceIndex != signature.npos && "Invalid signature");

		if (namespaceIndex == nameIndex)
			return "";

		auto namespaceView = signature.substr(nameIndex + 1, namespaceIndex - nameIndex);
		std::string outNamespace;
		std::size_t prevPos = 0;

		for (std::size_t pos = 0; pos < namespaceView.size(); ++pos) {
			if (namespaceView[pos] == '@') {
				outNamespace += "::" + std::string(namespaceView.substr(prevPos, pos - prevPos));
				prevPos = pos + 1;
			}
		}

		return outNamespace.substr(2);
	}

	constexpr std::string GetCallingConvention(const std::string_view& signature) noexcept {
		assert(signature.size() > 5 && "Invalid signature size");
		assert(signature.starts_with('?') && "Signature must start with '?'");
		assert(signature.ends_with("Z") && "Signature must end with 'Z'");

		auto namespaceIndex = signature.find("@@");
		assert(namespaceIndex != signature.npos && "Invalid signature");

		switch (signature[namespaceIndex + 3]) {
			case 'A': return "__cdecl";
			case 'X': return "__stdcall";
			case 'F': return "__fastcall";
			case 'D': return "__vectorcall";
			default: return "__unknown";
		}
	}

	constexpr std::string GetPrimitiveType(
		const std::string_view& view,
		std::size_t* position = nullptr
	) noexcept {
		assert(!view.empty() && "Invalid signature");

		const auto SetPosition = [&position](std::size_t pos) {
			if (position) *position = pos;
		};

		switch (view[0]) {
			case 'D': SetPosition(1); return "char";
			case 'F': SetPosition(1); return "short";
			case 'H': SetPosition(1); return "int";
			case 'J': SetPosition(1); return "long";
			case 'M': SetPosition(1); return "float";
			case 'N': SetPosition(1); return "double";
			case 'O': SetPosition(1); return "long double";
			case '_':
				switch (view[1]) {
					case 'N': SetPosition(2); return "bool";
					case 'J': SetPosition(2); return "long long";
				}
				break;
			case '?': {
				std::string modifier;
				switch (view[1]) {
					case 'A': modifier = ""; break;
					case 'B': modifier = "const "; break;
					case 'C': modifier = "volatile "; break;
					case 'D': modifier = "const volatile "; break;
				}
				std::size_t pos = 0;
				auto res = GetPrimitiveType(view.substr(2), &pos);
				SetPosition(pos + 2);
				return modifier + res;
			}
			case 'P':
			case 'Q':
			case 'R':
			case 'S': {
				assert(view.size() > 3 && view[1] == 'E' && "Invalid signature");

				std::string out;
				switch (view[2]) {
					case 'A': out = ""; break;
					case 'B': out = "const "; break;
					case 'C': out = "volatile "; break;
					case 'D': out = "const volatile "; break;
				}
				
				std::size_t pos = 0;
				out += GetPrimitiveType(view.substr(3), &pos);
				SetPosition(pos + 3);

				switch (view[0]) {
					case 'P': out += "*"; break;
					case 'Q': out += "* const"; break;
					case 'R': out += "* volatile"; break;
					case 'S': out += "* const volatile"; break;
				}
				return out;
			}
			case 'T':
			case 'U': {
				assert(view.size() > 3 && view[1] == 't' && "Invalid signature");

				auto index = view.find("@");
				
				std::string type;
				switch (view[0]) {
					case 'T': type = "union "; break;
					case 'U': type = "struct "; break;
				}
				auto index2 = view.find("@", index + 1);
				SetPosition(index2 + 1);
				return type + std::string(view.substr(1, index - 1));
			}
			case 'X': {
				SetPosition(1);
				return "void";
			}
			case 'Z': {
				SetPosition(1);
				return "...";
			}
		}
		SetPosition(1);
		return "unknown";
	}

	constexpr std::string GetReturnType(const std::string_view& signature) noexcept {
		assert(signature.size() > 5 && "Invalid signature size");
		assert(signature.starts_with('?') && "Signature must start with '?'");
		assert(signature.ends_with("Z") && "Signature must end with 'Z'");

		auto namespaceIndex = signature.find("@@");
		assert(namespaceIndex != signature.npos && "Invalid signature");

		return GetPrimitiveType(signature.substr(namespaceIndex + 4));
	}

	constexpr std::string GetParamsType(const std::string_view& signature) noexcept {
		assert(signature.size() > 5 && "Invalid signature size");
		assert(signature.starts_with('?') && "Signature must start with '?'");
		assert(signature.ends_with("Z") && "Signature must end with 'Z'");

		auto namespaceIndex = signature.find("@@");
		assert(namespaceIndex != signature.npos && "Invalid signature");

		std::size_t pos = 0;
		GetPrimitiveType(signature.substr(namespaceIndex + 4), &pos);
		pos += namespaceIndex + 4;
		std::string out;

		std::vector<std::string> params;
		std::string_view view = signature.substr(pos);
		pos = 0;

		while (!view.empty()) {
			if (view[0] == 'Z') {
				if (view.size() == 1)
					break;
				params.push_back("...");
				break;
			}
			auto res = GetPrimitiveType(view.substr(pos), &pos);
			params.push_back(res);
			view = view.substr(pos);
			pos = 0;
		}
		for (auto i = 0; i < params.size(); i++) {
			if (!out.empty())
				out += ", ";
			out += params[i];
		}

		return out;
	}

	constexpr std::string GetDemangledName(const std::string_view& signature) noexcept {
		auto returnType = GetReturnType(signature);
		auto callingConvention = GetCallingConvention(signature);
		auto namespaceName = GetNamespaceName(signature);
		auto exportName = GetExportName(signature);
		auto paramsType = GetParamsType(signature);

		return returnType + " " + callingConvention + " " +
			namespaceName + "::" + exportName + "(" + paramsType + ")";
	}
}
