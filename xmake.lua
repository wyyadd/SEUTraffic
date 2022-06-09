set_project("SEUTraffic")
set_version("1.0.0")
set_xmakever("2.6.5")
set_languages("cxx11")

-- add_requires("vcpkg::gtest", {optional = true})

if is_mode("debug") then
		-- 添加DEBUG编译宏
		add_defines("DEBUG")
		-- 启用调试符号
		set_symbols("debug")
		-- 禁用优化
		set_optimize("none")
		set_warnings("all", "error")
else 
		-- 隐藏符号
		set_symbols("hidden")
		-- strip所有符号
		set_strip("all")
		set_optimize("fastest")
end

target("SEUTraffic_lib")
    set_kind("static")
    add_syslinks("pthread")
    add_files("src/**.cpp|seutraffic.cpp")
    add_includedirs("src", {public = true})
    add_includedirs("extern/rapidjson/include", {public = true})
    add_includedirs("extern/milo", {public = true})

target("simple_run")
    set_kind("binary")
    add_files("tools/debug/*.cpp")
    add_deps("SEUTraffic_lib")
