package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	private GeographicPoint location;
	List<MapEdge> edges;
	
	public MapNode(){
		edges = new ArrayList<MapEdge>();
	}
	
	public MapNode(GeographicPoint location){
		this.location = location;
		edges = new ArrayList<MapEdge>();
	}
	
	public GeographicPoint getLocation(){
		return location;
	}
	public void setLocation(GeographicPoint geo){
		location = geo;
	}
}
