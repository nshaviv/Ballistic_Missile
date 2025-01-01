using GLMakie 
using Makie
#import GeometryBasics
using FileIO  

# close any open screen if any are open
GLMakie.closeall() 

# Load the Earth's texture (if not already downloaded)

update_file = true
# dark
#texture_url = "https://eoimages.gsfc.nasa.gov/images/imagerecords/57000/57752/land_shallow_topo_2048.jpg"

# light low res
#texture_url = "https://eoimages.gsfc.nasa.gov/images/imagerecords/147000/147190/eo_base_2020_clean_720x360.jpg"

# light high res
texture_url = "https://eoimages.gsfc.nasa.gov/images/imagerecords/147000/147190/eo_base_2020_clean_3600x1800.png"

texture_path = "earth_texture.jpg"
if !isfile(texture_path) || update_file
    Base.download(texture_url, texture_path)
end

earth_sphere = GLMakie.Sphere(Point3f(0.0), R/1000.0)
earth_texture = load(texture_path);

function plot_makie_solution(xs, ys, zs; c = :blue)
        fig = Figure(size = (800,800))
        axs = Axis3(fig[1,1], aspect = :data)
        Makie.mesh!(axs, earth_sphere, color = earth_texture )
        Makie.lines!(axs, -xs./1000, -ys./1000, zs./1000, color = c, linewidth = 2)
        fig
    return fig, axs
end

