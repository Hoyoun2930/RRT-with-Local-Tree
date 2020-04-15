# RRT-with-Local-Tree

## Abstract
어떤 이미지가 AR (Augmented Reality)로 생성된 것인지 아닌지 구분하는 것은 까다로운 일이다. AR로 생성된 이미지는 가상의 물체를 현실 세계에 투영한 것이라 판단의 기준이 될 레퍼런스 이미지가 없기 때문이다. 본 연구는 이 문제를 해결고자 새로운 AR 이미지 판별 모델을 제안한다. 이 모델은 기존의 이미지 판별에 사용되던 CNN (Convolution Neural Network) 모델과 본 연구에서 새로이 제안하는 HNN (Histogram Neural Network) 모델을 앙상블 하여 구현되었다. 모델을 학습시키기 위하여 250장의 AR 이미지와 250장의 일반 이미지를 세그먼테이션 하여 데이터를 생성했으며, 해당 데이터를 통하여 모델을 학습시킨 결과 84.0%의 높은 정확도를 보였다. 

### RRT ([Rapidly-exploring random tree](https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree))
![image](https://user-images.githubusercontent.com/62214506/78421533-5cf43600-7693-11ea-9335-67bcff85eb97.png)
![image](https://user-images.githubusercontent.com/62214506/78421535-5e256300-7693-11ea-9b3f-6b23708768ae.png)

연구의 목표 정확도와 학습 시간 등을 고려하였을 때 과도한 Deep Layer 구조는 불필요하다고 생각하여 3개의 Convolution Layer를 비롯한 Shallow Network로 구성하였다. 본 연구의 목표는 입력된 이미지가 AR인지 Real 이미지인지 판별하는 것이므로 2-class classification을 수행하게 된다. 따라서 본 CNN 구현에는 Binary Classification을 상정하고 Parameter 및 Layer 배치를 진행하였다. 학습 이미지는 300x300 해상도로 리사이징 되어 CNN에 입력되고, 여러 필터 레이어를 거쳐 최종적으로 Fully Connect Layer에 의해 Binary 값이 결과로 출력된다. 

### HNN (Histogram Neural Network)
![image](https://user-images.githubusercontent.com/62214506/78421538-5fef2680-7693-11ea-8ee1-7aeb26b9e2bb.png)

HNN 모델은 오브젝트의 히스토그램 데이터와 오브젝트를 제외한 배경의 히스토그램 데이터를 학습을 위한 Train Data로 사용한다. HNN의 레이어는 총 2개의 2D Convolution Layer와 1개의 Fully-Connected Layer로 구성하였다.

### Ensemble
CNN과 HNN을 통하여 학습된 두 가지 모델은 각각의 특장점이 있다. 그래서 두 모델이 잘 구분해내는 AR 이미지의 특징이 다른데, 이 두 모델의 장점을 합쳐 발전시키고자, 본 연구에서는 가중치를 둔 보팅 (Weighted voting) [방법](http://doi.org/10.1109/IJCNN.2009.5178708)의 아이디어를 사용하였다. 

## Result
   |CNN|HNN|Ensemble
---|---|---|---|
Validation Accuracy|57.9%|85.9%|84.0%|
Validation Loss|6.5526|0.4223|   |
Test Accuracy|66.9%|83.0%|   |
Test Loss|5.0457|0.7677|   |

이러한 결과는 히스토그램이 AR 이미지 고유의 특징을 더 잘 부각시켜 줄 수 있다는 점과, 학습 속도의 한계 때문에 CNN 모델의 레이어를 깊게 구성하지 못하여 성능의 한계를 들어낸 것으로 보인다. CNN 모델의 한계 때문에 보팅을 이용한 앙상블 기법 역시 HNN 모델을 단독으로 사용하는 경우보다 오히려 정확도가 약간 떨어지는 결과를 얻은 것으로 해석된다. CNN 모델을 개선한다면 보다 나은 성능의 AR 이미지 판별 모델을 구현할 수 있을 것으로 보인다.
