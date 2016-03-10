package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private MapNode startPoint;
	private MapNode endPoint;
	private String streetName;
	private String roadType;
	double distance;
	final static double DEFAULT_LENGTH = 0.1;
	public MapEdge(){
		
	}
	public MapEdge(String roadName, MapNode n1, MapNode n2) 
	{
		this(roadName, "", n1, n2, DEFAULT_LENGTH);
	}
	public MapEdge(String roadName, String roadType2, MapNode n1, MapNode n2, double length) {
		this.startPoint = n1;
		this.endPoint = n2;
		this.streetName = roadName;
		this.roadType = roadType2;
		this.distance = length;
	}
	public MapNode getStartPoint(){
		return startPoint;
	}
	public void setStartPoint(MapNode geo){
		startPoint = geo;
	}
	public MapNode getEndPoint(){
		return endPoint;
	}
	public void setEndPoint(MapNode geo){
		endPoint = geo;
	}
	public GeographicPoint getEnd(){
		return endPoint.getLocation();
	}
	public GeographicPoint getStart(MapNode geo){
		return startPoint.getLocation();
	}
	public String getStreetName(){
		return streetName;
	}
	public void setStreetsName(String street){
		streetName = street;
	}
	public String getRoadType(){
		return roadType;
	}
	public void setRoadType(String type){ 
		roadType = type;
	}
	public MapNode getOtherNode(MapNode node){
		if(node.equals(endPoint))
			return startPoint;
		else if(node.equals(startPoint))
			return endPoint;
		throw new IllegalArgumentException("Looking for " +
				"a point that is not in the edge");
	}
}
