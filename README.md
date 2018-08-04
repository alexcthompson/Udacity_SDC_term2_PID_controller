# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

# Reflection

## Formalities

The code compiles without warnings or errors, and the car can successfully drive a full lap around the track with the settings:

- `P` as `Kp = 0.06875`
- `I` as `Ki = 0.00125`
- `D` as `Kd = 0.75`
- throttle set to a constant value of `0.4`

My values may not completely match others as I decided to interpret the PID outputs as steering angles in radians, and then limit them to the range `[-0.4363323, 0.4363323]`, which corresponds to -25 and 25 degrees.  I then scaled them to the range `[-1, 1]`, so my coefficients may perform differently.

# Practical considerations of P, I, and D

After twiddling as a human with values, and then observing the *Twiddle* algorithm twiddle with the PID values, I see two competing dynamics in play:

1. You want the PID controller to be responsive enough to get the car back on track quickly, particularly in sharp turns.
2. However, the PID system has a tendency to generate reinforcing oscillations that can strengthen through positive feedback and send the car into the lake or up into the hills.

`P` is straightforward - the more you are off the line, the more it tries to bring you back.  `P`'s influence becomes pathological when the car is far off track, and there is too much time for `P`'s contribution to make yaw relative to the driving line very high, leading to bad overshoot.

`D`, as a measure of the rate of change, serves to slow the rate of change, by opposing rapidly changing `CTE`.  It can help moderate `P`'s pathology.  However, when the car overshoots rapidly, `D` can contribute to the car steering violently back.  `P` and `D` together can be a bad combination and I have yet to figure out how to counter and balance them properly.  In general it seems that setting `D` to be 10-15 times larger than `P` seems to be about right.

`I` fixes bias and controls drift in turns.  By integrating the error, it can correct for a slow moving bias in the steering mechanism.  As well, when you go wide in turns, which tends to happen, `I`'s effect is to keep that wide swing from taking you off track, helping `P`.

# Tuning hyper-parameters

I did some fiddling by hand to find hyper parameters that got me through the first two turns.  Then I got ambitious, and decided to try and implement an iterative version of Twiddle.  I used this to refine my PID values until I found ones I liked.  I decided not to mess with speed, and pegged the throttle at a constant 0.4.

My iterative Twiddle implementation is imperfect and in need of a serious refactor, but here is the idea:

1. Starting with crude values, run Twiddle across a short distance until you can get the `CTE^2` value below a certain threshold _per step_, say 0.2.
2. Now, lengthen the distance on which you evaluate each run.
3. And lower your threshold for _per step_ squared error.
4. Again, run Twiddle, selecting better and better values until you have an PID parameter set that meets your _per step_ standard.  This algorithm _graduates_ to a longer course, with a more stringent standard for _per step squared error_.
5. Iterate.

In this way I was able to find the above values, which tend to run the course nicely.

Along the way, I ran into a lot of problems that remind me of other machine learning problems, particularly ones you run across in deep learning, here are some:

1. You want to learn the PID values that perform best for the whole course.  As you find values that do well for the first part of the course, you now want to test them on later parts.  So you would prefer to assess them on this later part.  But this comes with a few problems:
  - You have to wait for the car to get the later part
  - You have to define what those later parts are.  You typically need to focus on doing the curves well, but there are plenty of straight sections on the track.
2. The PID values that do well in straights are not ideal for curves, and those that do well in curves are not ideal for straights.  You want your iterative implementation to learn curves, but not forget straights, and vice versa.  Tricky!
3. Controlling model standards was tricky.  An error rate feasible in one part of the track was totally unachievable in others, so you need to control your model evaluation and graduation carefully.  As well, you have to be careful in how you cultivate early models, since they fail quickly, you must select for models that do well over short distances.
4. As I progressed, I could start to see lots of weaknesses in the PID controller design, and had ideas to correct them.
  - One obvious one is that the PID controller does not factor in estimated time to crossing the center-track-line at the current yaw rate.  `D` looks at rate but not rate vs distance.  `P` looks at distance but not rate of closure.  Ideally it would be good to have a factor in the model that looks at the time to closure.
  - As you cook up enough of these factors to add to the PID model, you can see that an iterative learning model for these factors gets very complex.  You start to imagine learning techniques that are basically those found in ML and Deep Learning.
  - Which begs the question, why not upgrade the model a bit.
  - And since we have an explicit physical model for the cars path, why not make use of that.

That, I feel, is the genius of this project: it makes the motivation for the MPC clear.  It's very obvious that the MPC is a solution to 95% of the issues of the PID.
