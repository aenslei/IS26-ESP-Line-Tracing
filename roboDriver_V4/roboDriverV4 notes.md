roboDriverV4 notes, ask Ainsley if you're confused pls, no shy okay i am very chill :D

just phrase ur qn nicely can already thank you, preferably in the grp chat cuz i dont see tele DMs as fast for some reason LMAO



-------------------------------------------

TO TAKE NOTE WHEN DEV-ING:



1\. **FSM data structures**- there are 2 very impt ones that youll see on top of the roboDriverV4: sensor and fsm state. 

—> they store all your signal data and robot states respectively. 

—> **if there is any new signal or state, you *must remember to update these 2 structs* or else they will not be properly defined/called in the system**

--> if you need to call or fetch a signal value in Sensors, simply do "sensor.xxx", xxx being what you named the variable itself within the sensor struct.







2\. **process of adding a state, in order of steps**:



2.1. Add the states name under the FSM State data struct at the top. Please keep the state handler naming convention of "STATE\_XXX" when naming your state



2.2. Add whatever global static variables you need right under that. If you have new signal data \[eg USS/MQTT stuff], pls add inside sensor struct



2.3. Ctrl+F for State Handlers. scroll to the bottom of this list and add your newest state function \[this is where all state-related logic will be]. Please keep naming convention to "state\_handle\_XXX"


2.4. add this state inside the function "state\_process\_state" Line 859 onwards. **this switch case updates the state for you based on an update period in the main function**. DO NOT FORGET TO ADD IT HERE, OR THE STATES WILL NEVER BE REFRESHED PROPERLY !







3\. **how do states even fucking work?**

to break this down as simply as I can:



3.1. robot ALWAYS boots up in STATE\_INIT. this state initialises the robot. all init-related logic for all drivers is called in STATE\_INIT


3.2. from there, based on very clear conditions put in each state \[as u can see from all the if loops], the global constant "fsm\_state\_t current\_state" on Line 54 will change depending on what the bots supposed to do. this variable determines what state your machine is in, and is refreshed in main following an update period of xx/ms. \[I forgot.]



###### 3.3. ***states call each other, and all logic to transition states are found in state functions only "state\_handle\_xxx". main NEVER calls a state. Please remember this. If I catch anyone doing this and it breaks the damn thing I will not be happy with you bro***







4\. **what are some good rules to follow when developing for our FSM?**
based on my experience, a couple things to really try to remember:



4.1 **every piece of logic that involves moving the robot or doing some sort of math operation *should only be reachable under a condition* \[if loop]**. 

--> if you read a few states, youll notice almost all of them DO NOT have a 'normal' or 'catch-all' operation that tells the bot to do smth. all of them are locked behind conditions.

--> why? it makes sure that diff logic is not called accidentally. its only invoked when a specific condition is hit. this gives you as the developer explicit power over when the robot does what. use this power carefully

--> conditions are mostly data-driven. as you can see in all states, most conditions are based off of "sensor.xxx", which is our driver's data.



4.2  **when you plan state logic before you actly code it \[pls do ah...pls] please make sure the robots actions always loop back to STATE\_LINE\_FOLLOWING.**

--> why? this robots main goal is to line follow. *every other state is there to navigate the bot such that it will always return to STATE\_LINE\_FOLLOWING*
--> hence, pls make sure all your state logic will somehow point back to line following at the end of the day. u must plot out the logic and see if the robots action loop closes. 

--> if ur stuck, u gotta plan what logic or what data would cause the robot to start line following again. ask urself qns like:

"what data or signal does the robot need to know that it must line follow?" , 

"what math must the robot do to determine whether it shld line follow or not?" or 

"what state must the robot be in for it to know to transition back to line following?"

--> if ur confused, pls look at the state calls and logic for STATE\_CORRECTION\_LEFT and RIGHT. youll see what I mean by this



4.3. **please dont forget to add a return within every conditional loop after transitioning states**

--> to transition from state to state, use function "fsm\_transition\_to(STATE\_XXX)". yes, the name in the struct, NOT the state handler function name state\_handler\_abc
--> please add 'return;' after that so the code doesn't crash out like me.



4.4. **unless absolutely necessary \[pls consult me if ur not sure], there is no need to touch main.**

--> main's function is to process the newest state and update the sensor data by calling the drivers. there is, at my current understanding, *absolutely no need to touch main*

--> rn it's just add state, make sure state revolve back to line following. that is it



