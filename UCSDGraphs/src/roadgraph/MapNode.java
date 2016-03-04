package roadgraph;

import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	private GeographicPoint location;
	List<MapEdge> edges;
	public MapNode(){
		
	}
	public GeographicPoint getLocation(){
		return location;
	}
	public void setLocation(GeographicPoint geo){
		location = geo;
	}
}
