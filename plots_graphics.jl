using Plots

plotly()  # for interactive plots
#gr()     # for nicer static plots

#plot a wireframe sphere of earth
function wire_sphere(R)
    θ = LinRange(0, 2π, 15*4)
    ϕ = LinRange(0, π, 15*2)
    for i in 1:length(θ)
        x = R * cos.(θ[i]) .* sin.(ϕ)
        y = R * sin.(θ[i]) .* sin.(ϕ)
        z = R * cos.(ϕ)
        Plots.plot!(x, y, z, color=:gray, label=:none)
    end
    for i in 1:length(ϕ)
        x = R * cos.(θ) .* sin.(ϕ[i])
        y = R * sin.(θ) .* sin.(ϕ[i])
        z = R * cos.(ϕ[i]) .* ones(length(θ))
        Plots.plot!(x, y, z, color=:gray, label=:none)
    end
end

# plot the solution
function plot_sol(xs, ys, zs, label; newfig = true, c = :blue)
    if newfig Plots.plot() end 
    Plots.plot!(xlabel="x", ylabel="y", zlabel="z") 
    wire_sphere(R); 
    Plots.plot!(xs, ys, zs, label = label, lw = 3, color=c)
end
