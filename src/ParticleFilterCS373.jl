module ParticleFilterCS373
using Distributions

world_size = 100
landmarks  = [(20.0, 20.0), (80.0, 80.0), (20.0, 80.0), (80.0, 20.0)]

type Robot
  x::Float64
  y::Float64
  orientation::Float64
  forward_noise::Float64
  turn_noise::Float64
  sense_noise::Float64
end

function Robot()
  return Robot(rand()*world_size, rand()*world_size, rand()*2*π, 0., 0., 0.)
end

function set!(rob::Robot, new_x, new_y, new_orientation)
  if(new_x < 1 || new_x > world_size)
    throw(ArgumentError(@sprintf("X coordinate out of bounds: %s", new_x)))
  end
  if(new_y < 1 || new_y > world_size)
    throw(ArgumentError(@sprintf("Y coordinate out of bounds: %s", new_y)))
  end

  rob.x = new_x
  rob.y = new_y
  rob.orientation = new_orientation

  rob
end

function set_noise!(rob::Robot, new_f_noise, new_t_noise, new_s_noise)
  rob.forward_noise = new_f_noise
  rob.turn_noise = new_t_noise
  rob.sense_noise = new_s_noise

  rob
end

function evaluate(r::Robot, p)
  sum = 0.0
  for i = 1:length(p) # calculate mean error
    dx = (p[i].x - r.x + (world_size/2.0)) % world_size - (world_size/2.0)
    dy = (p[i].y - r.y + (world_size/2.0)) % world_size - (world_size/2.0)
    err = sqrt(dx * dx + dy * dy)
    sum += err
  end
  return sum / float(len(p))
end

function sense(rob::Robot)
  Z = Array(Float64, length(landmarks))
  for ((x, y), i) in zip(landmarks, [1:length(landmarks)])
    println(i)
    dist = sqrt((rob.x - x)^2 + (rob.y - y)^2)
    dist += rob.sense_noise == 0 ? 0 : rand(Normal(0, rob.sense_noise), 1)[1]
    Z[i] = dist
  end
  return Z
end

function move!(rob::Robot, turn, forward)
  if(forward < 0)
    throw(ArgumentError("Robot cannot move backwards"))
  end

  # turn, and add randomness to the turning command
  turn_noise   = rob.turn_noise == 0 ? 0 : rand(Normal(0, rob.turn_noise), 1)[1]
  orientation  = rob.orientation + turn + turn_noise
  orientation %= 2π

  # move, and add randomness to the motion command
  forward_noise = rob.forward_noise == 0 ? 0 : rand(Normal(0, rob.forward_noise), 1)[1]
  dist = forward + forward_noise
  x = rob.x + cos(orientation) * dist
  y = rob.y + sin(orientation) * dist
  x %= world_size
  y %= world_size

  # set particle
  res = Robot()
  set!(res, x, y, orientation)
  set_noise!(res, rob.forward_noise, rob.turn_noise, rob.sense_noise)

  return res
end

function Gaussian(μ, σ, x)
  return exp(- ((μ - x)^2) / σ^2 / 2 ) / sqrt(2π*σ^2)
end

function measurement_prob(rob::Robot, measurements)
  prob = 1.0
  for (x, y) in landmarks
    dist = sqrt((rob.x - x)^2 + (rob.y - y)^2)
  end
  return prob
end

end