
This is the final version of the code I created in September 2003 for my master internship.
The goal was to simulate a colony of robots that have to cross a gap bigger than the size of one robot.

In my work I highlighted three strategies. Each one has its own folder:

- fourmis: the simulation based on ant strategy.
- robots: the simulation based on robots with simple communication strategy.
- robots_int: the simulation based on pretty "evolved" robots.

To compile the source you need 3 additional libraries:

- ODE, the open dynamics engine. You can find it here: http://ode.org/
- OpenGL
- Glut, the OpenGL Utility Toolkit, you can find it here: https://www.opengl.org/resources/libraries/glut/

For information the library option for gcc is LIBS=-ldrawstuff -lode -lm -lstdc++ -lGL -lGLU

In the make file you need to edit the INCLUDE_REP and LIBS_REP variables to point to your includes and libs paths.

Please note that all the comments are in French...

