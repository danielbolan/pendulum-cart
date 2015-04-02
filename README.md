## Pendulum Cart
###### A toy simulation of a nonholonomic system

This was built with the intention that it would serve as a platform on which to build/play with different planners. The goal is to have a different planner in each branch.

The system itself is pretty simple: you have a cart on a frictionless track attached to a pendulum with a massless arm. You have two degrees of freedom (```x``` and ```theta```) and one input (force on the cart's body, ```u```). You can change some of the settings of the simulation, listed below with their default values:

```js
{
  // properties of the cart itself:
  cart : {
    M: 0.6, // mass of cart body, in kg
    m: 0.1, // (point) mass of pendulum, in kg
    l: 1    // length of arm, in meters
  },

  // properties of the visualization
  canvas : {
    // parent DOM element, in case you want to embed into a different page
    parentElem: document.body,
    // hopefully self-explanatory
    width: 600,
    height: 400
  }
}
```

#### Building a planner

In order to start planning, you need to create an object that contains a function named ```getNextControl``` that can take a state as input and output a control, expressed as a single number representing a force on the cart in newtons. You'll pass this object to ```Simulation.setPlanner``` before you tell it to run. If you have initialization code, add another function inside your planner object called ``initialize``.

```js
var center = 1;
var k = 5;
var springController  = {
  // if you have initialization code that needs to be run, add initialize()
  // initialize: function( state ){},
  getNextControl: function( state ) {
    return k * (center-state.x);
  }
};

var sim = new Simulation();
sim.setPlanner( springController );
sim.run();
```

The state that gets passed to your functions will be an object containing:
  * ```x```: the cart's position in meters (positive is to the right)
  * ```v```: your cart's velocity in meters per second
  * ```theta```: angle of the pendulum arm in radians (this is zero when the arm is pointing *down)*
  * ```omega```: angular velocity of the pendulum arm in radians per second
  * ```t```: time in seconds since the simulation started

------

It's hard to plan if you can't see what might happen next! There's ```Simulation.propagate``` for that. You hand it a state and a control and it will return the state that will result after one timestep (25ms).

```js
var sim = new Simulation();

var s = new State(); // current state
var u = 0;           // control we'll apply (0 Newtons)

var s_prime = sim.propagate( s, u ); // should get the same state back
// ...yes, I know it's a lame example. I'm sorry.
```

Positive force means accelerating to the right, and and theta is 0 when the pendulum is facing down. You really get to define you own goal this, but most people will aim to get x to be 0 and theta to be pi.

Most planners have some pretty heavy overhead. Most code typically runs in a single thread in your browser, and if your script takes too long it'll make your page hang and your browser will start bugging you about killing your script. If you have a function that will take a long time to run, consider using [web workers](https://developer.mozilla.org/en-US/docs/Web/API/Web_Workers_API/Using_web_workers).

#### Notes

This system is currently deterministic, but I've tried to set things up in such a way that it wouldn't be too bad to add probabilities to the observations and propagation.

This simulation uses RK4 for state propagation. I've heard that Runge-Kutta methods do not play well with conserved values, and that a symplectic integrator is better in these cases. I went with RK4 because I had never implemented it before and I understood it better. It seems to work well for most cases, though if you leave it running over the weekend errors do accrue. On the other hand, if you end up with a plan that requires three days to run I think you might have bigger problems to deal with.

Pull requests are always welcome.
