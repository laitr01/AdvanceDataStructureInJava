/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */

public class MapGraph {
	//Add your member variables here in WEEK 2
	
	HashMap<GeographicPoint, MapNode> vertices;
	HashSet<MapEdge> edges;

	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		//  Implement in this constructor in WEEK 2
		vertices = new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<MapEdge>();
		
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		// Implement this method in WEEK 2
		return vertices.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		// Implement this method in WEEK 2

		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		// Implement this method in WEEK 2
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 */
	public void addVertex(double latitude, double longitude)
	{
		GeographicPoint pt = new GeographicPoint(latitude, longitude);
		this.addVertex(pt);
	}
	/* was already in the graph, or the parameter is null).
	 */
	public void addVertex(GeographicPoint location)
	{
		//  Implement this method in WEEK 2
		MapNode n = vertices.get(location);
		if(n== null){
			n = new MapNode(location);
			vertices.put(location, n);
		}else{
			System.out.println("Warning: Node at location " + location +
					" already exists in the graph.");
		}
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */

	
	public void addEdge(double lat1, double lon1, double lat2, double lon2, String roadName, String roadType) throws IllegalArgumentException {

		//Implement this method in WEEK 2
		GeographicPoint loc1 = new GeographicPoint(lat1, lon1);
		GeographicPoint loc2 = new GeographicPoint(lat2, lon2);
		MapNode node1  = vertices.get(loc1);
		MapNode node2 = vertices.get(loc2);
		if (node1 == null)
			throw new NullPointerException("addEdge: pt1:"+loc1+"is not in graph");
		if (node2 == null)
			throw new NullPointerException("addEdge: pt2:"+loc2+"is not in graph");

		addEdge(node1, node2, roadName, roadType, MapEdge.DEFAULT_LENGTH);
	}
	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName,
			String roadType) {

		MapNode n1 = vertices.get(pt1);
		MapNode n2 = vertices.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, MapEdge.DEFAULT_LENGTH);
	}

	public void addEdge(GeographicPoint pt1, GeographicPoint pt2, String roadName, 
			String roadType, double length) {
		MapNode n1 = vertices.get(pt1);
		MapNode n2 = vertices.get(pt2);

		// check nodes are valid
		if (n1 == null)
			throw new NullPointerException("addEdge: pt1:"+pt1+"is not in graph");
		if (n2 == null)
			throw new NullPointerException("addEdge: pt2:"+pt2+"is not in graph");

		addEdge(n1, n2, roadName, roadType, length);
	}

	/** Given a point, return if there is a corresponding MapNode **/
	public boolean isNode(GeographicPoint point)
	{
		return vertices.containsKey(point);
	}

	private void addEdge(MapNode n1, MapNode n2, String roadName,
			String roadType,  double length)
	{
		MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
		edges.add(edge);
		n1.addEdge(edge);
	}
	public String testString(){
		return getNumVertices() + " vertices/" + getNumEdges() + " edges";
	}
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// Implement this method in WEEK 2
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		MapNode startPoint = vertices.get(start);
		MapNode goalPoint = vertices.get(goal);
		if (startPoint == null) {
			System.err.println("Start node " + start + " does not exist");
			return null;
		}
		if (goalPoint == null) {
			System.err.println("End node " + goal + " does not exist");
			return null;
		}
		HashSet<MapNode> visitedNode = new HashSet<>();
		HashMap<MapNode, MapNode> parentNodes  = new HashMap<>();
		Queue<MapNode> queue = new LinkedList<MapNode>();
		queue.add(startPoint);
		MapNode curr =null;
		boolean found = false;
		while(!queue.isEmpty()){
		
			//visitedNode.add(startPoint);
			curr = queue.remove();
			//nodeSearched.accept(curr.getLocation());
			if(curr.equals(goalPoint)){
				found = true;
				break;
			}
			Set<MapNode> neighbors = getNeighbors(curr);
			for(MapNode neighbor: neighbors){
				if(!visitedNode.contains(neighbor)){
					visitedNode.add(neighbor);
					parentNodes.put(neighbor, curr);
					queue.add(neighbor);
					
				}
			}
		
		}
		if (!curr.equals(goalPoint)) {
			System.out.println("No path found from " +start+ " to " + goal);
			return null;
		}
		// Hook for visualization.  See writeup.
		// Reconstruct the parent path
		List<GeographicPoint> path =
				reconstructPath(parentNodes, startPoint, goalPoint);

		return path;
		
	}

	private List<GeographicPoint> reconstructPath(HashMap<MapNode,MapNode> parentMap, MapNode start, MapNode goal)
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.println("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println(theMap.testString());
		System.out.println("DONE.");
		List<GeographicPoint> route = theMap.bfs(new GeographicPoint(1.0,1.0), 
				 new GeographicPoint(8.0,-1.0));

		System.out.println(route);
		route = theMap.bfs(new GeographicPoint(1.0,1.0), 
				 new GeographicPoint(4.0,1.0));

		System.out.println(route);
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
