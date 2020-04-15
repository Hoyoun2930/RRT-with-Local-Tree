# RRT-with-Local-Tree

## Abstract
RRT는 샘플링 기반 경로 알고리즘으로, 비홀로놈계에서 쉽게 경로계획이 가능한 알고리즘으로 인식되어왔습니다. 하지만 복잡한 맵에 대해서는 RRT 방법이 오랜 시간이걸린다는 단점이 있습니다. 본 연구에서는 이러한 단점을 해결하기 위한 알고리즘인, Local Tree의 효율을 보다 개선하고자 하였습니다. Local Tree는 시작 지점과 골 지점뿐만 아니라 랜덤한 점도 root로 하는 트리를 의미합니다. 본 연구에서는 Local Tree의 root를 Narrow passage(장애물 사이 좁은 경로)로 선택하는 방법을 적용하였습니다. 그 결과, Local Tree를 이용한 RRT 알고리즘의 수행 시간을 효과적으로 개선할 수 있음을 확인할 수 있었습니다.

## RRT ([Rapidly-exploring random tree](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree))
![RRT](https://user-images.githubusercontent.com/62214506/79302933-9bf27900-7f28-11ea-94fd-8c5c5cf19af7.png)

La Valle에 의해 설계된 RRT는 시작 지점에서부터 트리를 랜덤하게 생성해가며 골 지점까지 경로를 탐색하는 경로계획 알고리즘입니다. RRT는 정해진 구역 안에서 랜덤한 좌표의 점을 선택합니다. 그리고 트리에서 가장 가까운 노드를 선택한 뒤, 그 노드로부터 랜덤한 점을 향해 step size만큼 트리를 뻗어나갑니다. 만약 장애물에 의해 뻗어나가는 것이 불가능하다면 트리가 자라지 않습니다. 만약 골과 가장 가까운 노드와 골 사이의 거리가 스텝 사이즈 이하라면 골과 트리를 연결하고 시작지점으로부터 골까지의 경로를 출력합니다.

### [Local Tree](https://ieeexplore.ieee.org/document/1308756)
![Local Tree](https://user-images.githubusercontent.com/62214506/79307329-b11fd580-7f31-11ea-8529-d1c0b8eae598.png)

Local Tree란 시작 지점만을 root로 하고 트리를 뻗어나가는 것이 아니라 골 지점과 랜덤한 점도 root로 하는 트리를 의미합니다. 랜덤 점으로 기존 트리가 뻗어나가지 못할 경우, 일정 확률로 그 점에서 새로운 트리를 생성시키고, RRT와 똑같이 성장하며 트리끼리 만날 경우, 두 트리를 합치는 RRT입니다.

## Select local tree root
![Narrow Passage1](https://user-images.githubusercontent.com/62214506/79307335-b2e99900-7f31-11ea-9e1f-21df6c9845de.png)

Local Tree의 장점은 맵의 푸른 부분과 같은 좁은 부분을 탐색하는데 좋다는 것입니다. 하지만 Local Tree의 root로 붉은 부분에서 점을 택한다면 오히려 좋지 않은 결과가 발생할 수 있습니다. 이에 Local Tree의 장점을 최대한 살리기 위하여 Local Tree의 root로 Narrow passage를 잡는 방법에 대하여 고안해 보았으며 장점을 극대화 시켜 RRT의 효율을 높일 수 있을 것이라 판단하였습니다. 

### Min Length Method
![Narrow Passage2](https://user-images.githubusercontent.com/62214506/79307337-b2e99900-7f31-11ea-8894-6b8177ab8c6d.png)

본 연구에서는 Narrow passage를 한 점을 기준으로 양 쪽이 막혀있고 다른 방향이 뚫려있는 곳을 Narrow passage라고 정의하였으며 Narrow passage를 찾아 root로 선택하는 알고리즘을 Min Length Method라 정의하였습니다. 자세한 과정은 다음과 같습니다.

  1. 무작위로 한 점을 잡는다. 그 점을 q_rand라 한다.
  2. (1)에서 찾은 점 q_rand 로부터 뻗어나갈 방향을 무작위로 결정한다.
  3. q_rand 로부터 (2)에서 결정한 방향과 반대 방향으로 장애물과 걸릴 때까지 노드를 뻗어나가며 장애물에 걸릴 때까지 뻗은 노드의 길이 stepsize를 Narrow Passage의 가중치로 한다. 만약 일정 길이만큼 뻗어나가도 장애물에 걸리지 않을 경우 가중치를 Max로 한다.
  4. (1)에서 (3) 과정을 n번 반복하여 가중치가 낮은 점 k를 선택해 Local Tree의 root로 선택한다.

## Result
![result](https://user-images.githubusercontent.com/62214506/79308506-aebe7b00-7f33-11ea-8936-e3ede81c731b.png)

Basic Local Tree 보다 Min Length Method를 이용한 Local Tree가 iteration number가 평균적으로 52.7%정도 적은 것을 확인할 수 있었습니다. 따라서 본 연구에서 설계한 Min Length Method를 이용한 Local Tree가 Basic Local Tree 보다 시간 측면에서 효율적이라는 것을 볼 수 있었습니다.

결론적으로 본 연구는 Local Tree 알고리즘의 비효율적일 수 있는 랜덤하게 root를 선택하는 부분을 Min Length Method를 이용하여 Narrow passage를 root로 선택함으로써 Local Tree의 장점을 극대화 하였으며, 이를 이용한 RRT의 수행 시간을 단축 시켰습니다. 

