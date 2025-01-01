using DifferentialEquations
using LinearAlgebra

# Constants
g = 9.81 # m/s^2
R = 6371e3 # m
day = 24 * 60 * 60 - 4 * 60 # s
ω = 2π / day # rad/s
tmax = 90 * 60 # longest possible flight time

ang(v1, v2) = acos(min(1.0, (v1 ⋅ v2) / (norm(v1) * norm(v2))))

# equations of motion
function f(u, p, t)
    x, y, z, vx, vy, vz = u
    r = sqrt(x^2 + y^2 + z^2)
    # Coriolis force
    ax = 2 * ω * vy + ω^2 * x
    ay = -2 * ω * vx + ω^2 * y
    return [vx, vy, vz, -g * R^2 * x / r^3 + ax, -g * R^2 * y / r^3 + ay, -g * R^2 * z / r^3]
end

# integrate the trajectory and stop if the radius is less than R
# or if the radius is greater than 10R (the missile is lost)
function integrate_trajectory(x0, y0, z0, vx0, vy0, vz0, tmax)
    u0 = [x0, y0, z0, vx0, vy0, vz0]
    tspan = (0.0, tmax)
    affect!(integrator) = terminate!(integrator)
    condition(u, t, integrator) = (sqrt(u[1]^2 + u[2]^2 + u[3]^2) - R) * (sqrt(u[1]^2 + u[2]^2 + u[3]^2) - 10 * R)
    cb = ContinuousCallback(condition, affect!)
    callbk = CallbackSet(cb)
    prob = ODEProblem(f, u0, tspan)
    sol = solve(prob, Tsit5(), abstol=1e-6, reltol=1e-6, saveat=1.0, callback=callbk)
    return sol
end

function get_xyz_and_vs_from_init(v0, θ0, φ0; lat=0.0, lon=0.0)
    r = R * [cosd(lat) * cosd(lon), cosd(lat) * sind(lon), sind(lat)]
    up = r/norm(r)
    east_un = cross(up, [0, 0, 1])
    east = east_un/norm(east_un)
    north = cross(east, up)
    v = up*v0*sind(θ0) + v0*cosd(θ0)*(sind(φ0)*east + cosd(φ0)*north)
    return r, v
end 

function integration_from_initial(v0, θ0, φ0; lati=0.0, loni=0.0)
    r, v = get_xyz_and_vs_from_init(v0, θ0, φ0; lat=lati, lon=loni)    
    sol = integrate_trajectory(r..., v..., tmax)
    soli(i) = [(sol.u[j])[i] for j in 1:length(sol.u)] 
    # extract the solution
    ts = sol.t
    xs, ys, zs, vx, vy, vz = soli.([1,2,3,4,5,6])
    rs = [sqrt(x^2 + y^2 + z^2) for (x, y, z) in zip(xs, ys, zs)]
    return ts, xs, ys, zs, rs, vx, vy, vz
end
