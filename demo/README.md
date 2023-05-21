# Demo package
*this package contains a demo for the first client presentation*

## Functionality
The package implements a few of the essential building blocks that will be needed in the final system.
These include:

### Moving the base
A `/move_base` action server is already running in the base simulation. The server uses amcl planning in the background to move the base to a desired position in the map. 
The demo package implements a simple action client that sends a pose goal to the action server. 

