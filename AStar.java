package com.poojandpatel; /**
 * Authors: Hemang Hirpara & Poojan Patel
 * Introduction to Artificial Intelligence
 * 
 * A* Algorithm implemented on a MxM size grid with % of blocked cells
 * 
 * Repeated Forward A*, Repeated Backward A*, and Adaptive A* have been implemented
 */
import java.time.Duration;
import java.time.Instant;
import java.util.*;

public class AStar {

    // grid with knowledge of blocked cells
    private static Cell[][] real_world;
    
    // grid the agent will travel; initially not blocked
    private static Cell[][] agent_world;
    
    // Cost to move one of four compass directions
    private final int COST = 1;
    private static final int NUMBER_OF_GRIDS = 50;

    //50 gridworld
    private static ArrayList<Cell[][]> grids;

    //Dimensions of the gridworlds
    private static final int WIDTH = 101;
    private static final int HEIGHT = 101;

    //static boolean isForward;
    //static boolean isBackward;
    static boolean isAdaptive = false;
    static boolean useHigherGCosts = false;

    HashMap<Cell, Integer> adaptiveClosedList;
    
    //Collection of partial paths towards solution
    private static ArrayList<ArrayList<Cell>> paths;

    // Solution path
    private static ArrayList<Cell> solutionPath;
    
    // Start and end cells
    private static Cell start, end;


    /**
     * Set up the gridworlds, assigns hCost to agentWorld, initializes globals
     * @param sx - x-coordinate for start node
     * @param sy - y-coordinate for start node
     * @param ex - x-coordinate for end node
     * @param ey - y-coordinate for end node
     */
    public AStar(int sx, int sy, int ex, int ey, Cell[][] grid)
    {
        real_world = grid;
        agent_world = new Cell[WIDTH][HEIGHT];

        solutionPath = new ArrayList<>();
        paths = new ArrayList<>();

        start = new Cell(sx, sy);
        end = new Cell(ex, ey);

        if(isAdaptive)
            adaptiveClosedList = new HashMap<>();

        for(int i = 0; i < WIDTH; i++)
        {
            for(int j = 0; j < HEIGHT; j++)
            {
                agent_world[i][j] = new Cell(i, j);
                if (!isAdaptive)
                    agent_world[i][j].hCost = Math.abs(i - end.x) + Math.abs(j - end.y);
            }
        }

        agent_world[sx][sy] = start;
        real_world[sx][sy] = start;

        agent_world[ex][ey] = end;
        real_world[ex][ey] = end;
    }

    private static void generateGrids()
    {
        grids = new ArrayList<>();

        for(int n = 0; n < NUMBER_OF_GRIDS; n++)
        {
            Cell[][] grid_to_add = new Cell[WIDTH][HEIGHT];
            for (int i = 0; i < WIDTH; i++) {
                for (int j = 0; j < HEIGHT; j++) {
                        grid_to_add[i][j] = new Cell(i, j);
                    }
            }
            addBlocks(grid_to_add);
            grids.add(grid_to_add);
        }
    }

    private static void printGrid(Cell[][] grid) {
        for(int i = 0; i < grid.length; i++)
        {
            for(int j = 0; j < grid[i].length; j++)
            {
                if(grid[i][j] == (start))
                    System.out.printf("%-3s", "ST ");
                else if(grid[i][j] == (end))
                    System.out.printf("%-3s", "EN ");
                else if(grid[i][j] != null)
                    System.out.printf("%-3s", "0 ");
                else
                    System.out.printf("%-3s", "BL ");

            }
            System.out.println();
        }
        System.out.println();
    }


    /**
     * Make 30% of the real world cells blocked cells
     */
    private static void addBlocks(Cell[][] world)
    {
        Random rand = new Random();
        for(int i = 0; i < WIDTH; i++)
        {
            for(int k = 0; k < HEIGHT; k++)
            {
                int randValue = rand.nextInt(100);
                if(randValue < 30)
                    world[i][k] = null;
            }
        }

    }

    /** 
     * Apply a possible path generated from agents world onto real world
     * Maintains a List of all partial paths found towards solution
     * @param s - Cell at which A* will start
     * @return The cell before a block or null if no path
     */
    private Cell applyPathA(Cell s) {
    	ArrayList<Cell> path_to_try;
    	if(isAdaptive) {
            path_to_try = AdaptiveAStar(s);
        }
		else
			path_to_try = AStarAlgorithm(s);
    	
        Cell agent = null, newStart = null;
        for(int i = 0; i < path_to_try.size();i++){
            agent = path_to_try.get(i);
            
            if(agent.x - 1 >= 0 && real_world[agent.x-1][agent.y] == null)
                agent_world[agent.x-1][agent.y] = null;
            if(agent.y - 1 >= 0 && real_world[agent.x][agent.y-1] == null)
                agent_world[agent.x][agent.y-1] = null;
            if(agent.x + 1 < WIDTH && real_world[agent.x+1][agent.y] == null)
                agent_world[agent.x+1][agent.y] = null;
            if(agent.y + 1 < HEIGHT && real_world[agent.x][agent.y+1] == null)
                agent_world[agent.x][agent.y+1] = null;
            
            if(real_world[agent.x][agent.y] == null)
            {
                agent_world[agent.x][agent.y] = null;
            	if(i != 0)
                	newStart = path_to_try.get(i-1);
                else
                    return null;
                break;
            }
        }

        path_to_try.subList(path_to_try.indexOf(agent),path_to_try.size()).clear();
        paths.add(path_to_try);
        return newStart;
    }

    private Cell applyPathB(Cell s)
    {
    	Cell beginning = end;
    	Cell ending = s;
    	ArrayList<Cell> path_to_try;

    	path_to_try = AStarAlgorithmB(beginning, ending);
		
        Cell agent = null, newStart = null;
        for(int i = 0; i < path_to_try.size();i++){
            agent = path_to_try.get(i);
            if(agent.x - 1 >= 0 && real_world[agent.x-1][agent.y] == null)
                agent_world[agent.x-1][agent.y] = null;
            if(agent.y - 1 >= 0 && real_world[agent.x][agent.y-1] == null)
                agent_world[agent.x][agent.y-1] = null;
            if(agent.x + 1 < WIDTH && real_world[agent.x+1][agent.y] == null)
                agent_world[agent.x+1][agent.y] = null;
            if(agent.y + 1 < HEIGHT && real_world[agent.x][agent.y+1] == null)
                agent_world[agent.x][agent.y+1] = null;
            
            if(real_world[agent.x][agent.y] == null)
            {
            	if(i != 0)
            	{
            		newStart = path_to_try.get(i-1);
            		break;
            	}
            	else
            		return null;
            }
        }

        path_to_try.subList(path_to_try.indexOf(agent),path_to_try.size()).clear();
        paths.add(path_to_try);
        return newStart;
    }
    /** 
     * Runs A* on agent world until a path is not found
     */
    private void repA()
    {
        Cell temp = start;
        do {
            temp = applyPathA(temp);
        } while(temp != null);
    }

    private void repB() {
    	Cell temp = start;
    	do {
    		temp = applyPathB(temp);
    	} while(temp!=null);
    }
    
    /**
     * Forward A* finds a path from Start Node to End Node, storing partial paths
     * When final path or cell is null, create the full path or there is no solution 
     
     * A* Algorithm - executed on agent world and maintains
     * knowledge of observed blocked cells in the four compass directions
     * @param s - Target Cell which A* will find a path to from START NODE
     * @return ArrayList path from Cell s to End node
     */
    private ArrayList<Cell> AStarAlgorithm(Cell s)
    {
        // Path to return
        ArrayList<Cell> path = new ArrayList<>();
        // Priority Queue to hold the cells yet to be expanded upon
        PriorityQueue<Cell> openList =  new PriorityQueue<>();
        // HashSet to hold cells which have been processed or are blocked off
        List<Cell> closedList = new ArrayList<>();
        resetCosts();
        s.gCost = 0;
        s.fCost = 0;
        openList.add(s);
        while(!openList.isEmpty())
        {
        	//This breaks ties by smaller g values
        	int lowestCost = openList.peek().fCost;
        	HashMap<Integer,Cell> ties = new HashMap<Integer,Cell>();
        	ArrayList<Integer> values = new ArrayList<Integer>();
        	for(Cell c : openList)
        	{
        		if(c.fCost == lowestCost)
        		{
        			ties.put(c.gCost,c);
        			values.add(c.gCost);
        		}
        	}

        	if(ties.size() > 1)
            {
                if(useHigherGCosts)
                {
                    Collections.sort(values, Collections.reverseOrder());
                    for(int i : values){
                        openList.remove(ties.get(i));
                        openList.add(ties.get(i));
                    }
                }
                else
                {
                    Collections.sort(values);
                    for(int i : values)
                    {
                        openList.remove(ties.get(i));
                        openList.add(ties.get(i));
                    }
                }
            }

        	
        	Cell curr = openList.poll();
            closedList.add(curr);

            if(curr.equals(end))
            	break; // go find the path

            // find all adjacent neighbors to check
            HashSet<Cell> successors = new HashSet<>();

            if(curr.x - 1 >= 0)
                successors.add(agent_world[curr.x - 1][ curr.y]);
            if(curr.y - 1 >= 0)
                successors.add(agent_world[curr.x][ curr.y-1]);
            if(curr.x + 1 < agent_world.length)
                successors.add(agent_world[curr.x + 1][ curr.y]);
            if(curr.y + 1 < agent_world[0].length)
                successors.add(agent_world[curr.x][ curr.y+1]);
            
            for(Cell child : successors)
            {
                if(closedList.contains(child) || child == null)
                    continue;
                   /*
                int newG = curr.gCost + COST;
                if(!openList.contains(child) || newG < child.gCost)
                {
                    child.gCost = newG;
                    child.hCost = agent_world[child.x][child.y].hCost;
                    child.fCost = child.gCost + child.hCost;
                    child.parent = curr;
                    if(!openList.contains(child))
                        openList.add(child);
                    else
                    {
                        openList.remove(child);
                        openList.add(child);
                    }
                }
                 */
                   int cost = curr.gCost + COST;

                if(openList.contains(child) && cost < child.gCost)
                    openList.remove(child);

                if(!openList.contains(child) && !closedList.contains(child))
                {
                    child.gCost = cost;
                    child.hCost = agent_world[child.x][child.y].hCost;
                    child.fCost = child.gCost + child.hCost;
                    child.parent = curr;
                    openList.add(child);
                }

            }
        }
        // COMPUTE THE PATH NOW
        Cell temp = end;
        while(temp != null){
            path.add(temp);
            temp = temp.parent;
        }
        //reverse path and return
        Collections.reverse(path);
        return path;
    }
    
    /**
     * Backward A* finds a path from End Node to the Start Node, storing partial paths once ->
     * agent follows said path from start to end, find a new path when agent hits a block or end
     * when you restart Backward A*, you search from newStart to the Start node
     *                                          from End Node to newStart
     * A* identical to regular A*, however, the it begins at the End Node
     * and finds a path to the Start Node 
     * @param ending - Target Cell which A* will find a path to from END NODE
     * @return ArrayList path from Cell s to End node
     */
    private ArrayList<Cell> AStarAlgorithmB(Cell beginning, Cell ending)
    {
    	// Path to return
        ArrayList<Cell> path = new ArrayList<>();
        // Priority Queue to hold the cells yet to be expanded upon
        PriorityQueue<Cell> openList =  new PriorityQueue<>();
        // HashSet to hold cells which have been processed or are blocked off
        List<Cell> closedList = new ArrayList<>();
        resetCosts();
        beginning.gCost = 0;
        //beginning.fCost = 0;



        openList.add(beginning);
        while(!openList.isEmpty())
        {
            // BREAK TIES
            int lowestCost = openList.peek().fCost;
            HashMap<Integer, Cell> ties = new HashMap<Integer, Cell>();
            ArrayList<Integer> values = new ArrayList<Integer>();
            for(Cell c : openList)
            {
                if(c.fCost == lowestCost)
                {
                    ties.put(c.gCost, c);
                    values.add(c.gCost);
                }
            }

            if(ties.size() > 1){
                Collections.sort(values, Collections.reverseOrder());
                for(int i : values){
                    openList.remove(ties.get(i));
                    openList.add(ties.get(i));
                }
            }

        	Cell curr = openList.poll();
            closedList.add(curr);
            if(curr.equals(ending))
            	break; // go find the path

            // find all adjacent neighbors to check
            HashSet<Cell> successors = new HashSet<>();

            if(curr.x - 1 >= 0)
                successors.add(agent_world[curr.x-1][curr.y]);
            if(curr.y - 1 >= 0)
                successors.add(agent_world[curr.x][curr.y-1]);
            if(curr.x + 1 < agent_world.length)
                successors.add(agent_world[curr.x+1][curr.y]);
            if(curr.y + 1 < agent_world[0].length)
                successors.add(agent_world[curr.x][curr.y+1]);
            
            for(Cell child : successors)
            {
                if(closedList.contains(child) || child == null)
                    continue;
                /*
                int newG = curr.gCost + COST;
                if(!openList.contains(child) || newG < child.gCost)
                {
                    child.gCost = newG;
                    child.hCost = agent_world[child.x][child.y].hCost;
                    child.fCost = child.gCost + child.hCost;
                    child.parent = curr;
                    if(!openList.contains(child))
                        openList.add(child);
                    else
                    {
                        openList.remove(child);
                        openList.add(child);
                    }
                }
                 */
                int cost = curr.gCost + COST;

                if(openList.contains(child) && cost < child.gCost)
                    openList.remove(child);

                if(!openList.contains(child) && !closedList.contains(child))
                {
                    child.gCost = cost;
                    child.hCost = agent_world[child.x][child.y].hCost;
                    child.fCost = child.gCost + child.hCost;
                    child.parent = curr;
                    openList.add(child);
                }


            }
        }
        // COMPUTE THE PATH NOW
        Cell temp = ending;
        while(temp != null){
            path.add(temp);
            temp = temp.parent;
        }
        // no need to reverse for Backwards, only if you want to view the paths being found backwards
        //System.out.println(path.toString());
        //printAgentWorld(path);
        return path;
    }
    	
    /**
     * Adaptive A* finds a path from Start Node to End Node, storing partial paths, identical to Repeated Forward A*
     * Only difference is if a cell has been visited, the heuristic cost is the difference of the previous executions goal - current
     * @param s - Cell at which A* will start
     * @return ArrayList path from Cell s to End Node
     */
    private ArrayList<Cell> AdaptiveAStar(Cell s)
    {
        // path to return
        //System.out.println("In adaptive star");
        ArrayList<Cell> path = new ArrayList<>();
        PriorityQueue<Cell> openList = new PriorityQueue<>();
        HashSet<Cell> closedList = new HashSet<>();
        resetCosts();
        s.gCost = 0;
        s.hCost = Math.abs(s.x - end.x) + Math.abs(s.y - end.y);
        s.fCost = s.gCost + s.hCost;
        openList.add(s);
        while(!openList.isEmpty())
        {
            // BREAK TIES
            int lowestCost = openList.peek().fCost;
            HashMap<Integer, Cell> ties = new HashMap<Integer, Cell>();
            ArrayList<Integer> values = new ArrayList<Integer>();
            for(Cell c : openList)
            {
                if(c.fCost == lowestCost)
                {
                    ties.put(c.gCost, c);
                    values.add(c.gCost);
                }
            }
            /* sort by lowest gCost
            if(ties.size() > 1){
                Collections.sort(values);
                for(int i : values){
                    openList.remove(ties.get(i));
                    openList.add(ties.get(i));
                }
            }

             */
            //sort by highest gCost
            if(ties.size() > 1){
                Collections.sort(values, Collections.reverseOrder());
                for(int i : values){
                    openList.remove(ties.get(i));
                    openList.add(ties.get(i));
                }
            }

            Cell curr = openList.poll();
            //System.out.println("Curr: " + curr);
            closedList.add(curr);
            adaptiveClosedList.put(curr, curr.gCost);

            
            if(curr.equals(end))
                break;

            // Get Neighbors
            HashSet<Cell> successors = new HashSet<>();
            if(curr.x - 1 >= 0) { successors.add(agent_world[curr.x - 1][ curr.y]); }
            if(curr.y - 1 >= 0) { successors.add(agent_world[curr.x][ curr.y-1]); }
            if(curr.x + 1 < WIDTH) { successors.add(agent_world[curr.x + 1][ curr.y]); }
            if(curr.y + 1 < HEIGHT) { successors.add(agent_world[curr.x][ curr.y+1]); }

            for(Cell child : successors)
            {
                if(closedList.contains(child) || child == null)
                    continue;

                /*
                int cost = curr.gCost + COST;
                if(child.gCost != -1){
                    System.out.println("end.gCost: " + end.gCost);
                    child.hCost = end.gCost - curr.gCost;
                }
                else
                    child.hCost = Math.abs(child.x - end.x) + Math.abs(child.y - end.y);
                 */

                int cost = curr.gCost + COST;
                int newHCost = Math.abs(child.x - end.x) + Math.abs(child.y - end.y);
                if(adaptiveClosedList.containsKey(child))
                {
                    newHCost = end.gCost - adaptiveClosedList.get(child);
                    //System.out.println("adaptive H used");
                }

                //System.out.println("Manhattan H used");



                if(openList.contains(child) && cost < child.gCost)
                    openList.remove(child);

                /* never executed
                if(closedList.contains(child) && cost < child.gCost)
                {
                    System.out.println("IN THE CLOSED LIST AND GONNA BE REMOVED");
                    closedList.remove(child);
                }
                 */

                if(!openList.contains(child) && !closedList.contains(child))
                {
                    child.gCost = cost;
                    child.hCost = newHCost;
                    child.fCost = cost + child.hCost;
                    child.parent = curr;
                    openList.add(child);
                }
            }
        }

        Cell temp = end;
        while(temp != null){
            path.add(temp);
            temp = temp.parent;
        }
        Collections.reverse(path);
        //System.out.println("Path: " + path.toString());
        //printGCosts();
        //printHCosts();
        //printAgentWorld(path);

        return path;
    }
    
    /**
     * Clears the data in the Cells of agent world
     * DOES NOT REMOVE BLOCKED CELLS
     */
    private void resetCosts()
    {
        for(int i = 0; i < WIDTH; i++)
            for(int j = 0; j < HEIGHT; j++)
            {
                //reset g and f costs
                if(agent_world[i][j] != null)
                {
                    if(!isAdaptive)
                    {
                        agent_world[i][j].gCost = 0;
                        agent_world[i][j].fCost = 0;
                        agent_world[i][j].parent = null;
                    }
                    else
                    {
                        agent_world[i][j].parent = null;
                    }

                }
            }
    }


    /**
     * Print out real grid in initial state
     */
    private void printSolution() 
    {
    	System.out.println("REAL WORLD");
        System.out.println("------------------------------");
        for(int i = 0; i < WIDTH; i++)
        {
            for(int j = 0; j < HEIGHT; j++)
            {
                if(i == start.x && j == start.y)
                    System.out.print("ST ");
                else if(i == end.x && j == end.y)
                    System.out.print("EN ");
                else if(real_world[i][j] != null)
                    System.out.printf("%-3s", real_world[i][j].isSolution ? "X " : "0 ");
                else
                    System.out.print("BL ");
            }
            System.out.println();
        }
        System.out.println("------------------------------");

        System.out.println("INITIAL G COSTS");
        for(int i = 0; i < WIDTH; i++)
        {
            for (int j = 0; j < HEIGHT;j++)
            {
                if(i == start.x && j == start.y)
                    System.out.print("ST ");
                else if(i == end.x && j == end.y)
                    System.out.print("EN ");
                else
                    System.out.printf("%-3s", agent_world[i][j].gCost);
            }
            System.out.println();
        }

    }

    private void printGCosts()
    {
        System.out.println("\nFinal G COSTS");
        for(int i = 0; i < WIDTH; i++)
        {
            for (int j = 0; j < HEIGHT;j++)
            {
                if(i == start.x && j == start.y)
                    System.out.print("ST ");
                else if(i == end.x && j == end.y)
                    System.out.print("EN ");
                else if(agent_world[i][j] != null)
                    System.out.printf("%-3s", agent_world[i][j].gCost);
                else
                    System.out.print("BL ");
            }
            System.out.println();
        }
    }

    private void printHCosts()
    {
        System.out.println("\nFinal H COSTS");
        for(int i = 0; i < WIDTH; i++)
        {
            for (int j = 0; j < HEIGHT;j++)
            {
                if(i == start.x && j == start.y)
                    System.out.print("ST ");
                else if(i == end.x && j == end.y)
                    System.out.print("EN ");
                else if(agent_world[i][j] != null)
                    System.out.printf("%-3s", agent_world[i][j].hCost);
                else
                    System.out.print("BL ");
            }
            System.out.println();
        }
    }

    /*
     * Concatenate of a List of List of Cells
     */
    private void concatPath() {
        for(List<Cell> path : paths)
        	for(Cell c : path)
            	solutionPath.add(c);
        /*
        for(int i = 0; i < solutionPath.size()-1; i++)
        {
        	int firstDup = solutionPath.indexOf(solutionPath.get(i));
        	int secondDup =solutionPath.lastIndexOf(solutionPath.get(i));
        	solutionPath.subList(firstDup,secondDup).clear();
        }
        */
    }

    /** Print out grid from the view of agent
     * @param path - Possible solution path highlighted on grid via *
     */
    private void printAgentWorld(ArrayList<Cell> path) {
    	String[][] agent_wrld = new String[WIDTH][HEIGHT];
        for(int i = 0; i < WIDTH; i++)
        {
            for(int j = 0; j < HEIGHT; j++)
            {
            	if(agent_world[i][j] == null)
            		agent_wrld[i][j] = "BL ";
            	else if(i == start.x && j == start.y)
            		agent_wrld[i][j] = "ST ";
            	else if(i == end.x && j == end.y)
            		agent_wrld[i][j] = "EN ";
            	else
            		agent_wrld[i][j] = "0  ";
            }
        }
        //System.out.println(solutionPath.toString());
        for(Cell c : path)
        {
        	agent_wrld[c.x][c.y] = "*  ";
        }
        
        agent_wrld[start.x][start.y] = "ST ";
        agent_wrld[end.x][end.y] = "EN ";
        System.out.println("AGENT WORLD");
        System.out.println("------------------------------");
        for(int i = 0; i < WIDTH; i++)
        {
            for(int j = 0; j < HEIGHT; j++)
            	System.out.print(agent_wrld[i][j]);
            System.out.println();
        }
        System.out.println("------------------------------");
    }

    /**
     * Print out formatted solution output
     */
    private void printOutput()
	{
		System.out.println("Success... Path Found\nGenerating Path from "+ paths.size()+" on Agent World...\n");
    	/*
		solutionPath.add(end);
    	System.out.println("SOLUTION PATH SIZE: " + (solutionPath.size()-1) + "\n");
    	System.out.println(solutionPath + "\n");
        printAgentWorld(solutionPath);
        System.out.println("\n***************************\n");
        */
	}

	private static boolean isSolution(){
        Cell endNode = end;
        int x = endNode.x;
        int y = endNode.y;
        Cell lastNodeInPath = solutionPath.get(solutionPath.size()-1);
        HashSet<Cell> neighbors = new HashSet<>();

        if(x-1 >= 0 && real_world[x-1][y] != null)
            neighbors.add(real_world[x-1][y]);
        if(y-1 >= 0 && real_world[x][y-1] != null)
            neighbors.add(real_world[x][y-1]);
        if(x+1 < WIDTH && real_world[x+1][y] != null)
            neighbors.add(real_world[x+1][y]);
        if(y+1 < WIDTH && real_world[x][y+1] != null)
            neighbors.add(real_world[x][y+1]);

        for(Cell n : neighbors)
        {
            if(n.equals(new Cell(lastNodeInPath.x, lastNodeInPath.y)))
                return true;
        }

        return false;



    }


    public static void main(String[] args)
    {
        generateGrids();
        int sx, sy, ex, ey;
        Random r = new Random();

        //times
        Double lowG = 0.0;
        Double highG = 0.0;
        Double backward = 0.0;
        Double adaptive = 0.0;

        //path size
        int lgp = 0;
        int hgp = 0;
        int bp = 0;
        int ap = 0;

        //number of successes
        int lgs = 0, hgs = 0, bs = 0, as = 0;

        Instant startT, endT;

        for(Cell[][] grid : grids)
        {
            System.out.println("\nITERATION: " + grids.indexOf(grid));
            sx = r.nextInt(WIDTH);
            sy = r.nextInt(WIDTH);
            ex = r.nextInt(WIDTH);
            ey = r.nextInt(WIDTH);

            System.out.println("start: [" + sx + ", " + sy + "]");
            System.out.println("end: [" + ex + ", " + ey + "]");

            // low G
            //System.out.println("Low G \n");
            AStar a = new AStar(sx,sy,ex,ey, grid);
            startT = Instant.now();
            a.repA();
            endT = Instant.now();
            lowG += Duration.between(startT, endT).toMillis();
            a.concatPath();
            lgp += solutionPath.size();
            //if(end.parent != null && solutionPath.size() > 0 && solutionPath.get(solutionPath.size()-1).equals(end.parent))
            if(isSolution())
                lgs++;
            //System.out.println(solutionPath.toString());
            //a.printAgentWorld(solutionPath);

            //high g
            //System.out.println("\nHigh G \n");
            useHigherGCosts = true;
            AStar b = new AStar(sx,sy,ex,ey, grid);
            startT = Instant.now();
            b.repA();
            endT = Instant.now();
            highG += Duration.between(startT, endT).toMillis();
            b.concatPath();
            hgp += solutionPath.size();
            //if(end.parent != null && solutionPath.size() > 0 && solutionPath.get(solutionPath.size()-1).equals(end.parent))
            if(isSolution())
                hgs++;
            //b.printAgentWorld(solutionPath);
            //System.out.println(solutionPath.toString());

            //backward
            //System.out.println("\nBackward\n");
            AStar c = new AStar(sx,sy,ex,ey, grid);
            startT = Instant.now();
            c.repB();
            endT = Instant.now();
            backward += Duration.between(startT, endT).toMillis();
            c.concatPath();
            bp += solutionPath.size();
            if(isSolution())
                bs++;
            //System.out.println(solutionPath.toString());

            //c.printAgentWorld(solutionPath);

            //adaptive
            //System.out.println("\nAdaptive\n");
            isAdaptive = true;
            AStar d = new AStar(sx,sy,ex,ey, grid);
            startT = Instant.now();
            d.repA();
            endT = Instant.now();
            adaptive += Duration.between(startT, endT).toMillis();
            d.concatPath();
            ap += solutionPath.size();
            //if(end.parent != null && solutionPath.size() > 0 && solutionPath.get(solutionPath.size()-1).equals(end.parent))
             if(isSolution())
                as++;
            //System.out.println(solutionPath.toString());


            isAdaptive = false;
            useHigherGCosts = false;
        }

        System.out.println("\nFINAL OUTPUTS \n");
        System.out.println("Lower G Pref took " + lowG + " ms" + " with avg path: " + lgp/NUMBER_OF_GRIDS + " success: " + lgs);
        System.out.println("Higher G Pref took " + highG + " ms" + " with avg path: " + hgp/NUMBER_OF_GRIDS + " success: " + hgs);
        System.out.println("Backward took " + backward + " ms" + " with avg path: " + bp/NUMBER_OF_GRIDS + " success: " + bs);
        System.out.println("Adaptive took " + adaptive + " ms" + " with avg path: " + ap/NUMBER_OF_GRIDS + " success: " + as);


        /*
        isAdaptive = true;
        AStar a = new AStar(0,0,9,9, grids.get(0));
        printGrid(real_world);

        a.repA();
        a.concatPath();
        a.printAgentWorld(solutionPath);
        System.out.println(solutionPath.toString());
        if(isSolution())
            System.out.println("SUCCESS");
        else
            System.out.println("FALSE");

         */


        return;
    }
}