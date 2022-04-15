# NarrowBot

Code for our qualifier bot: NarrowBot.  A pretty big improvement on our original, despite the only major change being to making the bot thinner.  It turns out that being able to easily access the shared hub is pretty valuable in Freight Frenzy, which was something our previous bot couldn't do.  In terms of general design, though, it more or less stayed the same: it's a claw attached to an arm.  The improved claw allowed it to be somewhat less precise in alignment when picking things up, but it was still pretty slow and needed a lot of practice.

Each folder contains EVERY source file present within the bot at that point in time.  I never deleted any of the excess code that was already on the hub, so here are the only files ever used by this bot in competition:

- PhoenixAutoBlueDuck.java
- PhoenixAutoRedDuck.java
- PhoenixAutoBlueWarehouse.java
- PhoenixAutoRedWarehouse.java
- EzrasLaw.java, main bot class
- WinningOpModeBlue.java, main teleop class for blue alliance
- WinningOpModeRed.java, main teleop class for red alliance

Most classes are still poorly documented, and there's even more confusing bloat in EzrasLaw than in the original BadBot.  It's hard to believe this bot actually did well at the qualifier.

narrowbot.jpg is a picture of NarrowBot depositing to the blue alliance hub if you want to see it.
