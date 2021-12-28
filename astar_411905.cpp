//2D grid problem using a* algorithm

#include<iostream>
#include<queue>
#include<stack>
#include<cmath>
#include<utility>
#include<cstring>
#include<cfloat>
using namespace std;

#define nrow 4 //No of rows in 2D grid
#define ncol 4 //No of columns in 2D grid
typedef pair<double,pair<int,int>> node; //to store f value of cell

//structure to store details like parent cell and f,g,h value to track path
struct tnode
{
    int p_i,p_j; //To store parent cell
    double f,g,h; //To store heuristic value
};

//function to check valid cell i.e to check out of indexing
bool checkValid(int i,int j)
{
    if(i>=0&&i<nrow&&j>=0&&j<ncol)
    {
        return true;
    }
    return false;
}

//function to check goal cell or not
bool checkGoal(pair<int,int>goal,int i,int j)
{
    //If goal state matched with current cell then return true.
    if(goal.first==i && goal.second==j)
    {
        return true;
    }
    return false;
}

//function to check blocked cell or not
bool checkUnblocked(int matrix[nrow][ncol],int i,int j)
{
    //If unblocked cell return true
    if(matrix[i][j]==1)
    {
        return true;
    }
    return false;
}

//Function to show the path
void showPath(pair<int,int>goal,tnode track_matrix[nrow][ncol])
{
    cout<<"Path from source to goal: \n";
    stack<pair<int,int>> stk;
    int row=goal.first;
    int col=goal.second;

    while(!(track_matrix[row][col].p_i==row && track_matrix[row][col].p_j==col))
    {
        stk.push(make_pair(row,col));
        int tr=track_matrix[row][col].p_i;
        int tc=track_matrix[row][col].p_j;
        row=tr;
        col=tc;
    }
    stk.push(make_pair(row,col));
    while(!stk.empty())
    {
    	pair<int,int>temp=stk.top();
    	stk.pop();
    	cout<<"->"<<"("<<temp.first<<","<<temp.second<<")"<<"  ";
	}
	cout<<endl;
	return;
}

//function to calculate heuristic value using euclidean distance
double getHFun(pair<int,int> goal,int i,int j)
{
    double result;
    result=sqrt(((i-goal.first)^2)+((j-goal.second)^2));
    return result;
}

//a* function
void aStarFun(int matrix[nrow][ncol],pair<int,int>source,pair<int,int>goal)
{
    int i,j;
    //If source cell is goal cell
    if(checkGoal(goal,source.first,source.second))
    {
        cout<<"Source cell is goal cell itself.\n";
        return;
    }
    bool isGoalFound=false;//Boolean type to check goal is found or not

    //matrix to store details of cell regarding parent cell and heuristic value
    tnode track_matrix[nrow][ncol];
    for(int p=0;p<nrow;p++)
    {
        for(int k=0;k<ncol;k++)
        {
            track_matrix[p][k].p_i=-1;
            track_matrix[p][k].p_j=-1;
            track_matrix[p][k].f=FLT_MAX;
            track_matrix[p][k].g=FLT_MAX;
            track_matrix[p][k].h=FLT_MAX;
        }
    }

    //fringe(open list) using priority queue
    priority_queue<node,vector<node>,greater<node>> fringe;
    fringe.push(make_pair(0.0,source));

    //Initializing parent cell for source
    track_matrix[source.first][source.second].p_i=source.first;
    track_matrix[source.first][source.second].p_j=source.second;
    track_matrix[source.first][source.second].g=0.0;

    //Closed list using hash-map to track which node has been explored or not
    bool closedList[nrow][ncol];

    //Initializing with false since initially no cell has been explored
    memset(closedList,false,sizeof(closedList));

    while(!fringe.empty())
    {
        //getting cell with low f value
        node n=fringe.top();
        double fcal,gcal,hcal;

        //Adding this node n to closed list as it is going to be explored.
        i=n.second.first;
        j=n.second.second;
        closedList[i][j]=true;

        //removing the node n from fringe
        fringe.pop();

        //Producing all the possible 8 successors of node n

        //left successor
        if(checkValid(i,j-1))
        {
            //If the left successor is goal
            if(checkGoal(goal,i,j-1))
            {
                isGoalFound=true;
                cout<<"Goal cell is found\n";
                track_matrix[i][j-1].p_i=i;
                track_matrix[i][j-1].p_j=j;
                showPath(goal,track_matrix);
                return;
            }
            /*
             if the successor is not in blocked state and not in closed list then
            */
            else if(checkUnblocked(matrix,i,j-1) && !closedList[i][j-1])
            {
                hcal=getHFun(goal,i,j-1);
                gcal=track_matrix[i][j].g+1.0;
                fcal=gcal+hcal;
                //If successor is not in open list, add it or update f value
                if(track_matrix[i][j-1].f>fcal || track_matrix[i][j-1].g==FLT_MAX)
                {
                   fringe.push(make_pair(fcal,make_pair(i,j-1)));

                    //update the details of new inserted cell into fringe
                   track_matrix[i][j-1].p_i=i;
                   track_matrix[i][j-1].p_j=j;
                   track_matrix[i][j-1].f=fcal;
                   track_matrix[i][j-1].g=gcal;
                   track_matrix[i][j-1].h=hcal;
                }
            }
        }

        //right successor
        if(checkValid(i,j+1))
        {
            //If the right successor is goal
            if(checkGoal(goal,i,j+1))
            {
                isGoalFound=true;
                cout<<"Goal cell is found\n";
                track_matrix[i][j+1].p_i=i;
                track_matrix[i][j+1].p_j=j;
                showPath(goal,track_matrix);
                return;
            }
            /*
             if the successor is not in blocked state and not in closed list then
            */
            else if(checkUnblocked(matrix,i,j+1) && !closedList[i][j+1])
            {
                hcal=getHFun(goal,i,j+1);
                gcal=track_matrix[i][j].g+1.0;
                fcal=gcal+hcal;
                //If successor is not in open list, add it or update f value
                if(track_matrix[i][j+1].f>fcal || track_matrix[i][j+1].g==FLT_MAX)
                {
                    fringe.push(make_pair(fcal,make_pair(i,j+1)));

                    //update the details of new inserted cell into fringe
                   track_matrix[i][j+1].p_i=i;
                   track_matrix[i][j+1].p_j=j;
                   track_matrix[i][j+1].f=fcal;
                   track_matrix[i][j+1].g=gcal;
                   track_matrix[i][j+1].h=hcal;
                }
            }
        }

        //top successor
        if(checkValid(i-1,j))
        {
            //If the top successor is goal
            if(checkGoal(goal,i-1,j))
            {
                isGoalFound=true;
                cout<<"Goal cell is found\n";
                track_matrix[i-1][j].p_i=i;
                track_matrix[i-1][j].p_j=j;
                showPath(goal,track_matrix);
                return;
            }
            /*
             if the successor is not in blocked state and not in closed list then
            */
            else if(checkUnblocked(matrix,i-1,j) && !closedList[i-1][j])
            {
                hcal=getHFun(goal,i-1,j);
                gcal=track_matrix[i][j].g+1.0;
                fcal=gcal+hcal;
                //If successor is not in open list, add it or update f value
                if(track_matrix[i-1][j].f>fcal || track_matrix[i-1][j].g==FLT_MAX)
                {
                    fringe.push(make_pair(fcal,make_pair(i-1,j)));

                    //update the details of new inserted cell into fringe
                   track_matrix[i-1][j].p_i=i;
                   track_matrix[i-1][j].p_j=j;
                   track_matrix[i-1][j].f=fcal;
                   track_matrix[i-1][j].g=gcal;
                   track_matrix[i-1][j].h=hcal;
                }
            }
        }

        //bottom successor(i+1,j)
         if(checkValid(i+1,j))
        {
            //If the bottom successor is goal
            if(checkGoal(goal,i+1,j))
            {
                isGoalFound=true;
                cout<<"Goal cell is found\n";
                track_matrix[i+1][j].p_i=i;
                track_matrix[i+1][j].p_j=j;
                showPath(goal,track_matrix);
                return;
            }
            /*
             if the successor is not in blocked state and not in closed list then
            */
            else if(checkUnblocked(matrix,i+1,j) && !closedList[i+1][j])
            {
                hcal=getHFun(goal,i+1,j);
                gcal=track_matrix[i][j].g+1.0;
                fcal=gcal+hcal;
                //If successor is not in open list, add it or update f value
                if(track_matrix[i+1][j].f>fcal || track_matrix[i+1][j].g==FLT_MAX)
                {
                    fringe.push(make_pair(fcal,make_pair(i+1,j)));

                    //update the details of new inserted cell into fringe
                   track_matrix[i+1][j].p_i=i;
                   track_matrix[i+1][j].p_j=j;
                   track_matrix[i+1][j].f=fcal;
                   track_matrix[i+1][j].g=gcal;
                   track_matrix[i+1][j].h=hcal;
                }
            }
        }

        //top-left successor(i-1,j-1)
        if(checkValid(i-1,j-1))
        {
            //If the top-left successor is goal
            if(checkGoal(goal,i-1,j-1))
            {
                isGoalFound=true;
                cout<<"Goal cell is found\n";
                track_matrix[i-1][j-1].p_i=i;
                track_matrix[i-1][j-1].p_j=j;
                showPath(goal,track_matrix);
                return;
            }
            /*
             if the successor is not in blocked state and not in closed list then
            */
            else if(checkUnblocked(matrix,i-1,j-1) && !closedList[i-1][j-1])
            {
                hcal=getHFun(goal,i-1,j-1);
                gcal=track_matrix[i][j].g+1.0;
                fcal=gcal+hcal;
                //If successor is not in open list, add it or update f value
                if(track_matrix[i-1][j-1].f>fcal || track_matrix[i-1][j-1].g==FLT_MAX)
                {
                    fringe.push(make_pair(fcal,make_pair(i-1,j-1)));

                    //update the details of new inserted cell into fringe
                   track_matrix[i-1][j-1].p_i=i;
                   track_matrix[i-1][j-1].p_j=j;
                   track_matrix[i-1][j-1].f=fcal;
                   track_matrix[i-1][j-1].g=gcal;
                   track_matrix[i-1][j-1].h=hcal;
                }
            }
        }

        //top-right successor(i-1,j+1)
        if(checkValid(i-1,j+1))
        {
            //If the top-right successor is goal
            if(checkGoal(goal,i-1,j+1))
            {
                isGoalFound=true;
                cout<<"Goaltrack_matrix is found\n";
                track_matrix[i-1][j+1].p_i=i;
                track_matrix[i-1][j+1].p_j=j;
                showPath(goal,track_matrix);
                return;
            }
            /*
             if the successor is not in blocked state and not in closed list then
            */
            else if(checkUnblocked(matrix,i-1,j+1) && !closedList[i-1][j+1])
            {
                hcal=getHFun(goal,i-1,j+1);
                gcal=track_matrix[i][j].g+1.0;
                fcal=gcal+hcal;
                //If successor is not in open list, add it or update f value
                if(track_matrix[i-1][j+1].f>fcal || track_matrix[i-1][j+1].g==FLT_MAX)
                {
                    fringe.push(make_pair(fcal,make_pair(i-1,j+1)));

                    //update the details of new inserted cell into fringe
                   track_matrix[i-1][j+1].p_i=i;
                   track_matrix[i-1][j+1].p_j=j;
                   track_matrix[i-1][j+1].f=fcal;
                   track_matrix[i-1][j+1].g=gcal;
                   track_matrix[i-1][j+1].h=hcal;
                }
            }
        }

        //left-bottom successor(i+1,j-1)
        if(checkValid(i+1,j-1))
        {
            //If the left-bottom successor is goal
            if(checkGoal(goal,i+1,j-1))
            {
                isGoalFound=true;
                cout<<"Goal cell is found\n";
                track_matrix[i+1][j-1].p_i=i;
                track_matrix[i+1][j-1].p_j=j;
                showPath(goal,track_matrix);
                return;
            }
            /*
             if the successor is not in blocked state and not in closed list then
            */
            else if(checkUnblocked(matrix,i+1,j-1) && !closedList[i+1][j-1])
            {
                hcal=getHFun(goal,i+1,j-1);
                gcal=track_matrix[i][j].g+1.0;
                fcal=gcal+hcal;
                //If successor is not in open list, add it or update f value
                if(track_matrix[i+1][j-1].f>fcal || track_matrix[i+1][j-1].g==FLT_MAX)
                {
                    fringe.push(make_pair(fcal,make_pair(i+1,j-1)));

                    //update the details of new inserted cell into fringe
                   track_matrix[i+1][j-1].p_i=i;
                   track_matrix[i+1][j-1].p_j=j;
                   track_matrix[i+1][j-1].f=fcal;
                   track_matrix[i+1][j-1].g=gcal;
                   track_matrix[i+1][j-1].h=hcal;
                }
            }
        }

        //right-bottom successor(i+1,j+1)
        if(checkValid(i+1,j+1))
        {
            //If the right-bottom successor is goal
            if(checkGoal(goal,i+1,j+1))
            {
                isGoalFound=true;
                cout<<"Goal cell is found\n";
                track_matrix[i+1][j+1].p_i=i;
                track_matrix[i+1][j+1].p_j=j;
                showPath(goal,track_matrix);
                return;
            }
            /*
             if the successor is not in blocked state and not in closed list then
            */
            else if(checkUnblocked(matrix,i+1,j+1) && !closedList[i+1][j+1])
            {
                hcal=getHFun(goal,i+1,j+1);
                gcal=track_matrix[i][j].g+1.0;
                fcal=gcal+hcal;
                //If successor is not in open list, add it or update f value
                if(track_matrix[i+1][j+1].f>fcal || track_matrix[i+1][j+1].g==FLT_MAX)
                {
                    fringe.push(make_pair(fcal,make_pair(i+1,j+1)));

                    //update the details of new inserted cell into fringe
                   track_matrix[i+1][j+1].p_i=i;
                   track_matrix[i+1][j+1].p_j=j;
                   track_matrix[i+1][j+1].f=fcal;
                   track_matrix[i+1][j+1].g=gcal;
                   track_matrix[i+1][j+1].h=hcal;
                }
            }
        }
    }
    if(!isGoalFound)
    {
    	cout<<"Goal cell not found!\n";
	}
	return;
}

//main function
int main()
{
    /*
    Creating grid where
	1 represent cell is not blocked i.e no hindrance
	0 represent cell is blocked.
	*/
	int matrix[nrow][ncol]={
                            {1,0,0,0},
                            {1,1,0,1},
                            {0,1,0,0},
                            {1,1,1,1}
                           };
    //Printing grid
    cout<<"Grid: \n";
    for(int i=0;i<nrow;i++)
    {
        for(int j=0;j<ncol;j++)
        {
            cout<<" "<<matrix[i][j];
        }
        cout<<endl;
    }
    cout<<endl;

    //User input for source cell
    pair<int,int> source;
    cout<<"Enter source cell: ";
    cin>>source.first>>source.second;

    //Invalid source cell
    if((!checkValid(source.first,source.second))&&(!checkUnblocked(matrix,source.first,source.second)))
    {
        cout<<"Source cell is invalid.\n";
        return 0;
    }

    //User input for destination cell
    pair<int,int> goal;
    cout<<"Enter destination(goal) cell: ";
    cin>>goal.first>>goal.second;

    //Invalid destination cell
    if((!checkValid(goal.first,goal.second))&&(!checkUnblocked(matrix,goal.first,goal.second)))
    {
        cout<<"Goal cell is invalid.\n";
        return 0;
    }

    aStarFun(matrix,source,goal);
    return 0;
}
