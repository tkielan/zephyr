set(IS_TEST 1)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(NONE)

enable_language(ASM)

target_sources(app PRIVATE src/main.c src/asmhelp.S)
