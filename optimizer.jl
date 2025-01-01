using Optim

function fitness(v0, θ0, φ0; lati=0.0, loni=0.0, latf=0.0, lonf=0.0)
    # find the location of the target
    r1, v1 = get_xyz_and_vs_from_init(v0, θ0, φ0; lat=latf, lon=lonf)
    # intigrate the trajectory
    ts, xs, ys, zs, rs, vx, vy, vz = integration_from_initial(v0, θ0, φ0; lati=lati, loni=loni)
    # find the distance to the target
    distance_to_target = norm([xs[end], ys[end], zs[end]] - r1)
    # return the fitness function
    return distance_to_target/1.0e6 + (v0/5000.0)^2 # minimize distance and kinetic energy
end


