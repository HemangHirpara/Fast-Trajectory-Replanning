package com.poojandpatel;

public class Cell implements Comparable{
	public int x, y;
	public Cell parent;
	// and H(n) the heuristic that estimates the cost of the cheapest path from n to the goal
	public int hCost;
	//G(n) the cost of the path from the start node to n
	public int gCost;
	// G + H
	public int fCost;

	public boolean isSolution; // if cell is part of the solution path

	public boolean equals(Cell that){
		if(this.x == that.x && this.y == that.y)
			return true;
		else
			return false;
	}

	public Cell(int x, int y)
	{
		this.x = x;
		this.y = y;

	}
	
	public String toString()
	{
		return "[" + x + "," + y + "]";
	}


	@Override
	public int compareTo(Object o) {
		Cell that = (Cell) o;
		return (int)((this.fCost) - (that.fCost));
	}
}
