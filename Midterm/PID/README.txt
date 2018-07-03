PID.zip:
	build/
		makefile
		pid.yaml
	src/	
		main.cpp
		pid.hpp
		pid.cpp

for Windows:
	put pid.hpp and pid.cpp into cvdrone/src
	put pid.yaml into cvdrone/build/vs2015
	replace cvdrone/src/main.cpp with main.cpp
	add pid.hpp and pid.cpp into Sources folders (see figure)
	
for Unix:
	put pid.hpp and pid.cpp into cvdrone/src
	put pid.yaml into cvdrone/build/unix
	replace cvdrone/src/main.cpp with main.cpp
	replace cvdrone/build/makefile with makefile
