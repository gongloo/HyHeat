import os

Import("env")

def generateCoverageInfo(source, target, env):
    for file in os.listdir("test"):
        os.system(".pio/build/native/program test/"+file)
    os.system("lcov -d .pio/build/native/ -c -o lcov_raw.info")
    os.system("lcov --remove lcov_raw.info '*/.pio/*' '*/g++-v*' -o lcov.info")
    os.system("genhtml -o coverage/ --demangle-cpp lcov.info")

env.AddPostAction(".pio/build/native/program", generateCoverageInfo)