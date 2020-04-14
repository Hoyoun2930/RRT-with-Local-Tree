#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <list>
#include <vector>
using namespace std;
#define DIM				2
#define STEPSIZE		1.0
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
int aroundobstacle(point& p,vector<sphere>& obs,int k,double distance);
double root_min_distance(point& p, list<list<point> >& loctree);
long long int index;
int main()
{
	srand(time(NULL));
	FILE* in = fopen("input.txt","r"); 
	FILE* out= fopen("result.txt","w");
	fprintf(out,"time\titeration number\tnode number\tlocal tree number\n");
	int t,T;
	fscanf(in,"%d",&T);
	for(t=0;t<T;t++)
	{
		double P_mergechk = 0.1;
		double P_grow = 0.1;
		int obs_num;
		fscanf(in,"%d",&obs_num);
		// input of obstacle
		vector<sphere> obstacle;
		for(int i=0;i<obs_num;i++)
		{
			sphere tmp_sphere;
			for(int j=0;j<DIM;j++)
				fscanf(in,"%lf",&tmp_sphere.loc.q[j]);
			fscanf(in,"%lf",&tmp_sphere.r);
		}
		//input of starting point & ending point
		list< list<point> > loctree;
		point tmp_p;
		for(int i=0;i<DIM;i++)
			fscanf(in,"%lf",&tmp_p.q[i]);
		insert_loctree(tmp_p, loctree);
		for(int i=0;i<DIM;i++)
			fscanf(in,"%lf",&tmp_p.q[i]);
		insert_loctree(tmp_p, loctree);
		//init()
		int starting_time = clock();
		long long int node_num=2;
		index = 0;
		// local tree addition
		/*
		point pp;
		pp.q[0] = 0;
		pp.q[1] = 0;
		if(root_min_distance(pp,loctree) >20.0)
			insert_loctree(pp,loctree);
		*/
		// loctree.size() = local tree ����
		// RRT
		for(;;index++)
		{
			point now;
			now = random_point();
			node neighbor = nearest_neighbor(now,loctree);
			point now_last = now;								//default
			match_stepsize(now,*(neighbor.pointer)); 
			if(isinobstacle(now,obstacle))
			{
				if((rand()%1000)/1000.0 <= P_grow)				//default
				{												//default
					if(!isinobstacle(now_last,obstacle))		//default
						insert_loctree(now_last,loctree);		//default
				}												//default
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
		fprintf(out,"%d\t%lld\t%lld\t%d\n",ending_time-starting_time,index,node_num,loctree.size());
	}
	fclose(out);
	fclose(in);
	return 0;
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
	for(int i=0;i<obs.size();i++)
	{
		if(dist(p,obs[i].loc) < obs[i].r) return 1;
	}
	return 0;
}
double dist(point &p, point &q)
{
	double ret = 0;
	for(int i=0;i<DIM;i++) ret+= pow(p.q[i] - q.q[i],2.0);
	return sqrt(ret);
}
point random_point()
{
	point ret;
	for(int i=0;i<DIM;i++)
		ret.q[i] = (rand()%100) + (rand()%1000)/1000.0;
	return ret;
}
node nearest_neighbor(point& p, list<list<point> >& loctree)
{
	node ret;
	int loc = rand()%loctree.size();
	double distance=200.0;
	list<list<point> >::iterator it=loctree.begin();
	for(int i=0;i<loc;i++) it++;
	list<point>::iterator jt;
	for(jt = (*it).begin();jt!=(*it).end();jt++)
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
	for(int i=0;i<DIM;i++)
	{
		now.q[i] = p.q[i] + (now.q[i] - p.q[i])*STEPSIZE / distance;
	}
	return;
}
bool chkispath(list<point>::iterator& now, list<list<point> >::iterator& nowtree, list<list<point> >& loctree)
{
	list<list<point> >:: iterator start,end;
	start = loctree.begin();
	end = loctree.begin(); end++;
	list<list<point> >:: iterator it;
	for(it = loctree.begin();it!=loctree.end();it++)
	{
		if(it == nowtree) continue;
		list<point>::iterator jt;
		for(jt = (*it).begin();jt!=(*it).end();jt++)
		{
			if(dist(*now,*jt) < STEPSIZE)
			{
				if(nowtree == start && it == end )
				{
					pathexist(now,nowtree,jt,it);
					return 1;
				}
				else if(nowtree==end && it == start)
				{
					pathexist(jt,it,now,nowtree);
					return 1;
				}
				else if(nowtree==start || nowtree == end)
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
	list<point>::iterator it = end_node;
	list<point>::iterator bef = st_node;
	list<point>::iterator next;
	while(it != (*end).begin())
	{
		next = (*it).parent;
		(*it).parent = bef;
		bef = it;
		it = next;
	}
	next = (*it).parent;
	(*it).parent = bef;
	bef = it;
	it = next;
	(*st).merge(*end);
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
	return;
}
int aroundobstacle(point& p,vector<sphere>& obs,int k,double distance) // ���ϰ� = ��� ���ƴ���
{
	int ret = 0;
	point orient;
	for(int i=0;i<DIM;i++) orient.q[i] = 0.0;
	for(int i=0;i<k;i++)
	{
		point pt = p;
		point tmp;
		for(int j=0;j<DIM;j++) tmp.q[j] = (rand()%1000/1000.0) - 500.0;
		double leng = dist(orient,tmp);
		for(int j=0;j<DIM;j++) pt.q[j] += tmp.q[j]*distance/leng;
		if(isinobstacle(pt,obs)) ret++;
	}
	return ret;
}

double root_min_distance(point& p, list<list<point> >& loctree) // root_min_distance(pp, loctree)
{
	double ret = 200.0;
	list<list<point> >::iterator it;
	for(it = loctree.begin();it!=loctree.end();it++)
	{
		double tmp = dist(p,*((*it).begin));
		if(tmp < ret) ret = tmp;
	}
	return ret;
}