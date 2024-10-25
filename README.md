# Melty-brain Bots

A melty-brain bot is a style of [competition combat
robot](https://en.wikipedia.org/wiki/Robot_combat) in which the bot operates by
spinning constantly. There are two wheels, each rolling in opposite directions
to induce spin. By modulating the speed of each wheel, it is possible to control
the translational velocity of the robot (see [Control](#control) below). An
advantage of this approach is that the entire mass of the robot contributes to
the kinetic energy that can be delivered to an opponent, in comparison with
other designs where only part of the bot's [mass
budget](https://sparc.tools/SPARC_Robot_Construction_Specifications_v1.5.pdf) is
allocated to the weapon.

# Coordinate Frames

Bot frame, `+` denotes origin.
```
              +Y wheel
             /
           O         antenna
     .-----|-----.  /
     |           ||
     |     +     ||
     x           ||
   / '-----|-----'
LED        O
             \
              -Y wheel
 y
 ^
 |
 '---> x
```

Most of the GNC is defined in the arena (or "world") frame. The compass
directions (north, east, ...) are often used to describe this frame, though note
that these directions are unrelated to the earth's true/magnetic north.
```
                             arena y (north)
      O   . '\\    bot y _   ^
      .\'     \\        |\   |         _  bot x (p)
  . '          \\         \  |         .|
   \          . '          \ |     . '
    \     .\'               \| . '  ) theta
     x. '   O                '--------> arena x (east)
```

At boot, the bot frame and the arena frames have the same orientation. Theta is
defined as the counter-clockwise angle from arena x to bot x, in the arena
frame. As the bot spins counter-clockwise, theta increases.

Define the "pointing" vector `p = (px, py) = (cos(theta), sin(theta))`,
which is the unit vector in the arena frame with direction equal to bot x.

The rotation matrix from the bot to arena frame is:
```
bot_to_arena = [ cos(theta)  -sin(theta) ] = [ px  -py ]
               [ sin(theta)   cos(theta) ]   [ py   px ]
```

# Control

To translate, we modulate the speed of each wheel as the bot spins. Given a
desired direction of translation `d`, for any bot orientation one wheel will be
rolling towards `d` and the other will be rolling away*. "Towards" and "away"
are rough terms, meaning the angle between the vectors is less than or greater
than 90 degrees, respectively. At each instant, we want to increase the speed of
the wheel rolling towards `d` and decrease the speed of the wheel rolling away.
One wheel will be rolling towards `d` for half of one rotation of the bot, then
the wheels' roles will swap. Over the course of the half rotation where a given
wheel is being driven faster, the direction of the force it imparts on the bot
sweeps out a half circle. Integrating that contribution produces a net force
directly in the desired direction of motion.

To be precise, we should drive the -Y wheel faster when `p` points towards the
desired direction `d`, i.e. when `p . d > 0`. Otherwise, drive the +Y wheel
faster.

*There is the exception where both wheels are rolling exactly perpendicular to
the desired direction, i.e. `p . d = 0`.
