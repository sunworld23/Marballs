===============
MARBALLS README
===============

This readme, for the duration of this project, should serve as our agreement on various standards on this project.
Coding standards, formatting standards, and resources and references should be mentioned in this document.

CODING STANDARDS
================

	/!\ PROPOSED/UNOFFICIAL STANDARDS: (These can be included by general consensus or unspoken usage by multiple parties)
		a. Including a commented list of all functions in a file, in the order they appear, near the top of the file.
			(ex. /* Marble() - constructor \n MarbleShoot(marble marb) - shoots marbles \n Marblosion(marble marb, int strength) - blows things up */)
		b. Any debugging output lines should be prefaced with "[DEBUG] (filename.type) - "
			(ex. "[DEBUG] (particle.cpp) - Checking for crash point. Flying teapot not good sign.")
		
	NAMING CONVENTIONS:
		1. Class names and functions begin with capital letters, including contained words.
			(ex. Vector, VectorMath, Calculate())
		2. Variable names should use camelCasing. CONSTANTS ARE ALL CAPS, ALL THE TIME.
			(ex. bool marbleShooter, int bonzaiBalls, const int CRUISECONTROL = 1337)
		3. Variable names should reflect their purpose unless they are used in an iterator or loop counter.
			(ex. If a variable stores a normal vector, why not name it normVector or something?)
		4. Source code file names should be lower case. All file names should use underscores (_) instead of spaces.
	FORMATTING:
		1. Comments that fit should be inline using // so as to not use extravagant amounts of space.
			(ex. if (42 > 69) {...} // This checks if fundamental math has ceased working properly.)
		2. Functions should have a preceding comment including their name and purpose.
			(ex. // MarbleRoll - Rolls a marble so hard it becomes a biscuit.)
		3. Opening brackets { be placed on the line that beckoned them. Short functions may close on the very same line.
			(ex. if (README == "boring") {\n return "sorry for standards"; \n}, GetPie() { return pie; })
		4. Every code file should have a comment section on top denoting its name, purpose, date of last revision, and tasks to do.
			(ex. thing_stuffer.h \n Thing that does stuff. \n Last Revised: Tomorrow \n To Do: - Exist)
	
	Once a set of formatting standards are approved, anyone can check for inconsistencies and correct them (without nagging!)
	
RESOURCES AND REFERENCES
========================
	- Shove anything here you find useful that is linkable. It's nice to mention it here because github will notify of the change.
	- General advice is nice too, like tips for using github or git.
	
	LINKS OF INTEREST:
		- Tutorial's Source Code: https://github.com/idmillington/cyclone-physics
		- Link to Agilefant thing?
	
	GIT(HUB) ADVICE:
		- Github Notice: Before you make any changes to a file, use the pull command to get the most up to date versions of the file.
		