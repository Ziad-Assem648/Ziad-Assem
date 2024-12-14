import osmnx as ox
import heapq
from geopy.distance import great_circle
import folium
from tkinter import *
from tkinter import messagebox
from tkinter.ttk import *
import webbrowser
import traceback
from PIL import Image, ImageTk
from tkinter.ttk import Button, Entry
from geopy.geocoders import Nominatim



geolocator = Nominatim(user_agent="my_app") # type: ignore

print("Loading road networks...")

try:
    G_car = ox.graph_from_place("Aswan, Egypt", network_type="drive")

    G_foot = ox.graph_from_place("Aswan, Egypt", network_type="walk")

    print("Road networks loaded successfully!")
    print(f"Car Graph: {len(G_car.nodes)} nodes, {len(G_car.edges)} edges.")
    print(f"Foot Graph: {len(G_foot.nodes)} nodes, {len(G_foot.edges)} edges.")
except Exception as e:
    print("Error loading the road network graphs:", str(e))
    raise

def a_star_shortest_path(G, origin_coords, destination_coords):
    try:
        origin_node = ox.distance.nearest_nodes(G, origin_coords[1], origin_coords[0])
        destination_node = ox.distance.nearest_nodes(G, destination_coords[1], destination_coords[0])
        open_set = []
        heapq.heappush(open_set, (0, origin_node))
        came_from = {}
        g_costs = {origin_node: 0}

        while open_set:
            _, current_node = heapq.heappop(open_set)
            if current_node == destination_node:
                break

            for neighbor in G.neighbors(current_node):
                edge_data = G.get_edge_data(current_node, neighbor)
                if not edge_data or 'length' not in edge_data[0]:
                    continue
                travel_cost = edge_data[0]['length']
                tentative_g_cost = g_costs[current_node] + travel_cost

                if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g_cost
                    heuristic_cost = great_circle(
                        (G.nodes[neighbor]['y'], G.nodes[neighbor]['x']),
                        (G.nodes[destination_node]['y'], G.nodes[destination_node]['x'])
                    ).km
                    priority = tentative_g_cost + heuristic_cost
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current_node

        path = []
        current_node = destination_node
        while current_node != origin_node:
            path.append(current_node)
            current_node = came_from[current_node]
        path.append(origin_node)
        path.reverse()
        return path
    except Exception as e:
        print("Error calculating shortest path:", str(e))
        traceback.print_exc()
        raise

def generate_map_html(origin_coords, destination_coords, route_drone, route_car, route_foot, G_car, G_foot):
    m = folium.Map(location=origin_coords, zoom_start=16)

    folium.Marker(origin_coords, popup="Start Location", icon=folium.Icon(color="green")).add_to(m)
    folium.Marker(destination_coords, popup="Destination", icon=folium.Icon(color="red")).add_to(m)

    # Drone route (straight line)
    drone_layer = folium.FeatureGroup(name="Drone Route")
    folium.PolyLine([origin_coords, destination_coords], color="purple", weight=5, opacity=0.7).add_to(drone_layer)
    folium.Marker(
        location=[(origin_coords[0] + destination_coords[0]) / 2, (origin_coords[1] + destination_coords[1]) / 2],
        popup="Drone Path",
        icon=folium.Icon(color="purple", icon="cloud")
    ).add_to(drone_layer)
    drone_layer.add_to(m)

    # Car route (using G_car)
    if route_car:
        route_coords_car = [(G_car.nodes[node]['y'], G_car.nodes[node]['x']) for node in route_car]
        car_layer = folium.FeatureGroup(name="Car Route")
        folium.PolyLine(route_coords_car, color="blue", weight=5, opacity=0.7).add_to(car_layer)
        folium.Marker(
            location=[(origin_coords[0] + destination_coords[0]) / 2, (origin_coords[1] + destination_coords[1]) / 2],
            popup="Car Path",
            icon=folium.Icon(color="blue")
        ).add_to(car_layer)
        car_layer.add_to(m)

    # Foot route (using G_foot)
    if route_foot:
        route_coords_foot = [(G_foot.nodes[node]['y'], G_foot.nodes[node]['x']) for node in route_foot]
        foot_layer = folium.FeatureGroup(name="Foot Route")
        folium.PolyLine(route_coords_foot, color="green", weight=5, opacity=0.7).add_to(foot_layer)
        folium.Marker(
            location=[(origin_coords[0] + destination_coords[0]) / 2, (origin_coords[1] + destination_coords[1]) / 2],
            popup="Foot Path",
            icon=folium.Icon(color="green")
        ).add_to(foot_layer)
        foot_layer.add_to(m)

    folium.LayerControl().add_to(m)

    filename = "map_routes.html"
    m.save(filename)
    return filename

def find_route():
    try:
        origin_text = origin_entry.get().strip()
        destination_text = destination_entry.get().strip()

        origin_location = geolocator.geocode(origin_text)
        if origin_location is None:
            messagebox.showerror("Invalid Location", f"Could not find coordinates for origin: {origin_text}")
            return

        destination_location = geolocator.geocode(destination_text)
        if destination_location is None:
            messagebox.showerror("Invalid Location", f"Could not find coordinates for destination: {destination_text}")
            return

        origin_coords = [origin_location.latitude, origin_location.longitude]
        destination_coords = [destination_location.latitude, destination_location.longitude]
        route_car = a_star_shortest_path(G_car, origin_coords, destination_coords)
        route_foot = a_star_shortest_path(G_foot, origin_coords, destination_coords)
        route_drone = [origin_coords, destination_coords]  
        map_file = generate_map_html(origin_coords, destination_coords, route_drone, route_car, route_foot, G_car, G_foot)
        webbrowser.open(map_file)
        messagebox.showinfo("Success", "Routes have been calculated and displayed in your browser. You can switch between Drone, Car, and Foot routes from the layer control on the map.")
    except Exception as e:
        print("Error:", str(e))
        traceback.print_exc()
        messagebox.showerror("Error", f"An error occurred: {str(e)}")

# Tkinter Window
root = Tk()
root.title("Shortest Path Finder")
root.geometry("1300x600")
background_image = Image.open(r"C:/Users/EL-MOHANDES/Desktop/ddd/DALL·E 2024-12-11 01.43.16 - A machine learning model representation for determining the best route for a drone, car, and pedestrian. The image should show a futuristic digital ma.webp")
background_image = background_image.resize((1300, 600), Image.Resampling.LANCZOS)
background_photo = ImageTk.PhotoImage(background_image)
background_label = Label(root, image=background_photo)
background_label.place(relwidth=1, relheight=1)
Label(root, text="Enter Locations", font=("Helvetica", 14)).pack(pady=10)
frame = Frame(root)
frame.pack(pady=5)
Label(frame, text="Start Location:").grid(row=0, column=0, padx=5)
origin_entry = Entry(frame, width=30)
origin_entry.grid(row=0, column=1, padx=5)
Label(frame, text="Destination:").grid(row=1, column=0, padx=5)
destination_entry = Entry(frame, width=30)
destination_entry.grid(row=1, column=1, padx=5)
Button(root, text="Find Route", command=find_route).pack(pady=20)

root.mainloop()