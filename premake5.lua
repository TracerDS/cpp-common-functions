workspace "cpp-common-functions"
    configurations { "Debug", "Release" }
    platforms { "x86", "x64" }
    location "VS"
    cppdialect "c++latest"

project "cpp-common-functions"
    kind "ConsoleApp"
    language "C++"
    targetdir "VS/%{cfg.buildcfg}"

    files { "**.hpp" }
    vpaths {
        ["Headers"] = { "*.hpp" }
    }
    includedirs { "." }

    filter { "platforms:x86" }
        architecture "x86"

    filter { "platforms:x64" }
        architecture "x86_64"

    filter "configurations:Debug"
        defines { "DEBUG" }
        symbols "On"

    filter "configurations:Release"
        defines { "NDEBUG" }
        optimize "Off"
