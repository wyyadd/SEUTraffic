set_project("SEUTraffic")
set_version("1.0.0")
set_xmakever("2.6.5")
set_languages("cxx11")


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
    set_symbols("debug")