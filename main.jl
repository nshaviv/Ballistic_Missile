# Solution of the ballistic problem with the Coriolis force
# ----------------------------------------------------------
# The solution is carried out in the frame of reference of the Earth
# The Coriolis force is included in the equations of motion

include("solver.jl")
include("optimizer.jl")
include("plots_graphics.jl")
include("makie_graphics.jl")

# coordinates of the launch site.
# We'll use Sana'a, Yemen
lat0 = 15.3694 # degrees
lon0 = 44.1910 # degrees
# And aim for Tel-Aviv
lat1 = 32.0853 # degrees
lon1 = 34.7818 # degrees
# Or new york
lat1 = 40.7128 # degrees
lon1 = -74.0060 # degrees


# First, we solve the path of a problem with given initial conditions. (below we will find the optimal ones to hit a given target).

begin
    v0 = 7000.0 # m/s
    θ0 = 30.0 # degrees
    φ0 = 0.0 # degrees

    # To show the exaggerated effect of Coriolis, we'll use a 10 times larger value of ω
    ω = 2π / day * 10.0 
    ts, xs, ys, zs, rs, vx, vy, vz = integration_from_initial(v0, θ0, φ0; lati=lat0, loni=lon0)
    plot_sol(xs, ys, zs, "Rotating"; newfig=true, c=:blue)

    ω = 0.0 # neglect the Coriolis force
    ts0, xs0, ys0, zs0, rs0, vx0, vy0, vz0 = integration_from_initial(v0, θ0, φ0; lati=lat0, loni=lon0)
    plot_sol(xs0, ys0, zs0, "Static"; newfig=false, c=:red)
    display(Plots.plot!())

    ω = 2π / day  # back to the original value
end

# plot the path relative to the ground below the missile. 
begin 
    path_on_earth = R / 1000 .* [ang([x, y, z], [xs[1], ys[1], zs[1]]) for (x, y, z) in zip(xs, ys, zs)]
    Plots.plot(path_on_earth, (rs .- R) ./ 1000)
end

# optimize the launch parameters - Here we find the initial conditions that minimize the distance to the target and minimize the kinetic energy.
begin 
    ω = 2π / day # make sure the Coriolis force is included
    # optimize with initial guess of 4000 m/s, 45 degrees elevation and 0 degrees from the North.
    res = optimize(x -> fitness(x...; lati=lat0, loni=lon0, latf=lat1, lonf=lon1), [4000.0, 45.0, 0.0])
    # once we have a solution, we integrate the trajectory with the optimal initial conditions:
    ts, xs, ys, zs, rs, vx, vy, vz = integration_from_initial(res.minimizer...; lati=lat0, loni=lon0)
    fig, axs = plot_makie_solution(xs, ys, zs; c = :blue)
    
    ω = 0.0 # kill the coriolis force
    res0 = optimize(x -> fitness(x...; lati=lat0, loni=lon0, latf=lat1, lonf=lon1), [4000.0, 45.0, 0.0])
    ts0, xs0, ys0, zs0, rs0, vx0, vy0, vz0 = integration_from_initial(res0.minimizer...; 
                                                                      lati=lat0, loni=lon0)
    # Add the path to the plot
    Makie.lines!(axs, -xs0./1000, -ys0./1000, zs0./1000, color = :red, linewidth = 2)

    println("Coriolis effect on launch azimuth:", res0.minimizer[3]-res.minimizer[3])
    println("Max height of path:" , (maximum(rs)-R)/1000.0)
    println("Minimal v0: ", res.minimizer[1])
    println("Optimal angle of launch above the Horizon", res.minimizer[2])
end 

# Now we repeat the analysis, but instead of optimizing the trajectory with Coriolis, 
# we will optimize without coriolis, and then solve the trajectory with Coriolis but 
# the wrong initial conditions. This will tell us how much we will
# miss the target by neglecting the Coriolis force.
begin 
    ω = 0.0  # Find the parameters without Coriolis
    res = optimize(x -> fitness(x...; lati=lat0, loni=lon0, latf=lat1, lonf=lon1), [4000.0, 45.0, 0.0])
    ts0, xs0, ys0, zs0, rs0, vx0, vy0, vz0 = integration_from_initial(res.minimizer...; 
                                                                      lati=lat0, loni=lon0)    
    fig, axs = plot_makie_solution(xs0, ys0, zs0; c = :red)

    # Add coriolis but with the wrong (previous) initial conditions
    ω = 2π / day 
    ts, xs, ys, zs, rs, vx, vy, vz = integration_from_initial(res.minimizer...; lati=lat0, loni=lon0)
    Makie.lines!(axs, -xs./1000, -ys./1000, zs./1000, color = :blue, linewidth = 2)
    fig

    println("Final difference: ", norm([xs[end]-xs0[end], ys[end]-ys0[end], zs[end]-zs0[end]])/1000.0)
end 
