#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include<algorithm>
#include <time.h>
#include <math.h>
#include <list>
#include <vector>
using namespace std;
#define DIM 3
#define STEPSIZE 1.0
struct point
{
    double q[DIM];
    list<point>::iterator parent;
    bool operator < (const point&p)const
    {
        return q[0] < p.q[0];
    }
};

struct sphere
{
    point loc;
    double r;
};

struct node
{
    list<list<point> >::iterator tree;
    list<point>::iterator pointer;
};

void insert_loctree(point& st, list<list<point> >& loctree);
bool isinobstacle(point& p, vector<sphere>& obs);
double dist(point &p, point &q);
point random_point();
node nearest_neighbor(point& p, list<list<point> >& loctree);
void match_stepsize(point& now,point& p);
bool chkispath(list<point>::iterator& now, list<list<point> >::iterator& nowtree, list<list<point> >& loctree);
void mergetree(list<list<point> >& loctree, list<point>::iterator& st_node,list<list<point> >::iterator& st,list<point>::iterator& end_node,list<list<point> >::iterator& end);
void pathexist(list<point>::iterator start,list<list<point> >::iterator starttree, list<point>::iterator end,list<list<point> >::iterator endtree);
double root_min_distance(point& p, list<list<point> >& loctree);
int symaroundobstacle(point& p,vector<sphere>& obs,int k,int maxind);
long long int index;

int main()
{
    srand(time(NULL));
    FILE* in;
    FILE* out;

    in = fopen("obs1.txt","r");
    out= fopen("result1(ex1 k=1000 max=10).txt","w");

    fprintf(out,"time\titeration number\tnode number\tlocal tree number\tmax local tree number\n");
    int t,T;
    fscanf(in,"%d",&T);

    for(t=0; t<T; t++)
    {
        double P_mergechk = 0.1;
        double P_grow = 0.1;
        int obs_num;
        int max_numtree=0;
        fscanf(in,"%d",&obs_num);
        // input of obstacle
        vector<sphere> obstacle;
        for(int i=0; i<obs_num; i++)
        {
            sphere tmp_sphere;
            for(int j=0; j<DIM; j++)
                fscanf(in,"%lf",&tmp_sphere.loc.q[j]);
            fscanf(in,"%lf",&tmp_sphere.r);
            obstacle.push_back(tmp_sphere);
        }

//input of starting point & ending point
        list< list<point> > loctree;
        point tmp_p;
        for(int i=0; i<DIM; i++)
            tmp_p.q[i]=0.0;
        insert_loctree(tmp_p, loctree);

        for(int i=0; i<DIM; i++)
            tmp_p.q[i] = 100.0;

        insert_loctree(tmp_p, loctree);

//init()
        int starting_time = clock();
        long long int node_num=2;
        index = 0;
        point pp;
        vector<pair<int,point> >  pp_k;

        for(int i=0; i<1000; i++)
        {
            pp= random_point();
            if(!isinobstacle(pp,obstacle))
            {
                pp_k.push_back(pair<int,point>(symaroundobstacle(pp,obstacle,100,10),pp));
                /*printf("%d ",pp_k[i].first);*/
            }
        }

        sort(pp_k.begin(),pp_k.end());

        for(int i=0; i<pp_k.size(); i++)
        {
            if(pp_k[i].first<10)
                printf("%d ",pp_k[i].first);
        }

        printf("\n");

        for(int i=0; i<8&&i<pp_k.size(); i++)
        {
            if(pp_k[i].first<10)
                insert_loctree(pp_k[i].second,loctree);
        }
// RRT
        for(;; index++)
        {
            max_numtree = max(max_numtree,(int)loctree.size());
            point now;
            now = random_point();
            node neighbor = nearest_neighbor(now,loctree);
            point now_last = now;
            match_stepsize(now,*(neighbor.pointer));
            if(isinobstacle(now,obstacle))
            {
                continue;
            }
            node_num++;
            now.parent = neighbor.pointer;
            (*neighbor.tree).push_back(now);
            list<point>::iterator nowit = (*neighbor.tree).end();
            nowit--;

//chking mergetree
            if((rand()%1000) /1000.0 <= P_mergechk)
            {
                if(chkispath(nowit,neighbor.tree,loctree))
                    break;
            }
        }
        int ending_time = clock();
        fprintf(out,"%d\t%lld\t%lld\t%d\t%d\n",ending_time-starting_time,index,node_num,loctree.size(),max_numtree);
        printf("CASE #%3d\n",t+1);
        printf("time = %d\niteration number = %lld\nnode number = %lld\nlocal tree number = %d\nmax local tree number = %d\n",ending_time-starting_time,index,node_num,loctree.size(),max_numtree);
        printf("***************************************\n");
    }
}
void insert_loctree(point& st, list<list<point> >& loctree)
{
    list<point> tmp;
    tmp.push_back(st);
    loctree.push_back(tmp);
    return;
}
bool isinobstacle(point& p, vector<sphere>& obs)
{
    for(int i=0; i<DIM; i++)
    {
        if(p.q[i] > 100.0 || p.q[i]<0.0)
            return 1;
    }
    for(int i=0; i<obs.size(); i++)
    {
        if(dist(p,obs[i].loc) < obs[i].r)
            return 1;
    }
    return 0;
}

double dist(point &p, point &q)
{
    double ret = 0;
    for(int i=0; i<DIM; i++)
        ret+= pow(p.q[i] -   q.q[i],2.0);
    return sqrt(ret);
}

point random_point()
{
    point ret;
    for(int i=0; i<DIM; i++)
        ret.q[i] = (rand()%100) + (rand()%1000)/1000.0;
    return ret;
}

node nearest_neighbor(point& p, list<list<point> >& loctree)
{
    node ret;
    int loc = rand()%loctree.size();
    double distance=200.0;
    list<list<point> >::iterator   it=loctree.begin();
    for(int i=0; i<loc; i++)
        it++;
    list<point>::iterator jt;
    for(jt = (*it).begin(); jt!=(*it).end(); jt++)
    {
        double tmp = dist(p,*jt);
        if(tmp < distance)
        {
            distance = tmp;
            ret.tree = it;
            ret.pointer = jt;
        }
    }
    return ret;
}

void match_stepsize(point& now,point& p)
{
    double distance = dist(now,p);
    for(int i=0; i<DIM; i++)
    {
        now.q[i] = p.q[i] + (now.q[i] - p.q[i])*STEPSIZE / distance;
    }
    return;
}

bool chkispath(list<point>::iterator& now, list<list<point> >::iterator& nowtree, list<list<point> >& loctree)
{
    list<list<point> >:: iterator start,end;
    start = loctree.begin();
    end = loctree.begin();
    end++;
    list<list<point> >:: iterator it;
    for(it =   loctree.begin(); it!=loctree.end(); it++)
    {
        if(it == nowtree)
            continue;
        list<point>::iterator jt;
        for(jt = (*it).begin(); jt!=(*it).end(); jt++)
        {
            if(dist(*now,*jt) < STEPSIZE)
            {
                if(nowtree == start && it == end )
                {
                    //pathexist(now,nowtree,jt,it);
                    return 1;
                }
                else if(nowtree==end && it == start)
                {
                    //pathexist(jt,it,now,nowtree);
                    return 1;
                }
                else if(nowtree==start || nowtree ==   end)
                {
                    mergetree(loctree,now,nowtree,jt,it);
                    return 0;
                }
                else
                {
                    mergetree(loctree,jt,it,now,nowtree);
                    return 0;
                }
            }
        }
    }
    return 0;
}

void mergetree(list<list<point> >& loctree, list<point>::iterator& st_node,list<list<point> >::iterator& st,list<point>::iterator& end_node,list<list<point> >::iterator& end)
{
    st->splice(st->end(),*end);
    loctree.erase(end);
    return;
}

void pathexist(list<point>::iterator start,list<list<point> >::iterator starttree, list<point>::iterator end,list<list<point> >::iterator endtree)
{
    /*
	FILE* out = fopen("output.txt","w");
	vector<point> answer;
	while(start!=(*starttree).begin())
	{
		answer.push_back(*start);
		start = (*start).parent;
	}
	answer.push_back(*start);
	for(int i=answer.size()-1;i>=0;i--)
	{
		for(int j=0;j<DIM;j++) fprintf(out,"%lf ",answer[i].q[j]);
		fprintf(out,"\n");
	}
	while(end!=(*endtree).begin())
	{
		for(int j=0;j<DIM;j++) fprintf(out,"%lf ",(*end).q[j]);
		fprintf(out,"\n");
		end = (*end).parent;
	}
	for(int j=0;j<DIM;j++) fprintf(out,"%lf ",(*end).q[j]);
	fprintf(out,"\n");
	fclose(out);
	*/
}

double root_min_distance(point& p, list<list<point> >& loctree)
{
    double ret = 200.0;
    list<list<point> >::iterator it;
    for(it = loctree.begin(); it!=loctree.end(); it++)
    {
        double tmp = dist(p,*((*it).begin()));
        if(tmp < ret)
            ret = tmp;
    }
    return ret;
}
//Min Length Method
int symaroundobstacle(point& p,vector<sphere>& obs,int k,int maxind)
{
    int ret = INT_MAX;
    point orient;
    for(int i=0; i<DIM; i++)
        orient.q[i] = 0.0;
    for(int i=0; i<k; i++)
    {
        point pt1 = p,pt2 = p;
        point tmp;
        for(int j=0; j<DIM; j++)
            tmp.q[j] = (rand()%1000/1000.0) - 0.5;
        double leng = dist(orient,tmp);
        if(leng==0)
        {
            i--;
            continue;
        }
        for(int j=0; j<DIM; j++)
            tmp.q[j]= tmp.q[j]*STEPSIZE/leng;
        int tmpind=0;
        for(; tmpind<maxind; tmpind++)
        {
            for(int j=0; j<DIM; j++)
                pt1.q[j] += tmp.q[j];
            if(isinobstacle(pt1,obs))
                break;
        }
        for(; tmpind<maxind; tmpind++)
        {
            for(int j=0; j<DIM; j++)
                pt2.q[j] -= tmp.q[j];
            if(isinobstacle(pt2,obs))
                break;
        }
        ret = min(ret,tmpind);
    }
    return ret;
}
