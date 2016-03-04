package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	private GeographicPoint startPoint;
	private GeographicPoint endPoint;
	private String streetName;
	private String roadType;
	double distance;

	public MapEdge(){
		
	}
	public GeographicPoint getStartPoint(){
		return startPoint;
	}
	public void setStartPoint(GeographicPoint geo){
		startPoint = geo;
	}
	public GeographicPoint getEndPoint(){
		return endPoint;
	}
	public void setEndPoint(GeographicPoint geo){
		endPoint = geo;
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
}
