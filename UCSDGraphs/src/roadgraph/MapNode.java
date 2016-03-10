package roadgraph;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import geography.GeographicPoint;

public class MapNode {
	/** The list of edges out of this node */
	private HashSet<MapEdge> edges;
	private GeographicPoint location;
	private double distance;
	private double actualDistance;
	
	public MapNode(GeographicPoint location){
		this.location = location;
		edges = new HashSet<MapEdge>();
	}
	public Set<MapEdge> getEdges()
	{
		return edges;
	}
	Set<MapNode> getNeighbors()
	{
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}
	public GeographicPoint getLocation(){
		return location;
	}
	public void setLocation(GeographicPoint geo){
		location = geo;
	}
	// get node distance (predicted)
	public double getDistance() {
		return this.distance;
	}
	
	// set node distance (predicted)
	public void setDistance(double distance) {
	    this.distance = distance;
	}

	// get node distance (actual)
	public double getActualDistance() {
		return this.actualDistance;
	}
	
	// set node distance (actual)	
	public void setActualDistance(double actualDistance) {
	    this.actualDistance = actualDistance;
	}
	public void addEdge(MapEdge edge) {
		edges.add(edge);
		
	}
}
