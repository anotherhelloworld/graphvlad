\documentclass[12pt]{article}
\usepackage{graphicx}
\usepackage{amssymb}
\usepackage{listings}
\usepackage{amsmath}
\usepackage{graphicx}
\usepackage{float}
\usepackage[section]{placeins}
% Russian specicfic
% -------------------------
\usepackage[T2A]{fontenc}
\usepackage[utf8]{inputenc}
\usepackage[russian]{babel}
% -------------------------

\graphicspath{{./figs/}}

\begin{document}

\title{Общая связность сети. Критическое ребро.}

\author{
  Кирпа В. Д. (реализация, тестирование)
  \and
  Махлярчук Андрей (документация, тестирование)
  \and
  Утин Никита (визуализация, тестирование)
  \and
  Березкин Аркадий (реализация, документация)
}

\maketitle
\thispagestyle{empty}
\newpage

\tableofcontents
\listoffigures

\newpage

\section{Постановка задачи:}

\paragraph{}
Для графа $G = (V, E, W)$ с множеством вершин $V$,
множеством ребер $W: E \rightarrow \mathbb{R}_+$
найти ребро $e^*$, такое, что при замене
$W(e^*) \rightarrow \gamma W(e^*)$ сумма сетевых 
расстояний между всеми узлами минимизируется
(при $\gamma < 1$) или максимизируется (при $\gamma > 1$).

\section{Алгоритм}

\paragraph{}
Для нахождения суммы сетевых расстояний в графе использовался
алгоритм Дейкстры\cite{dijkstra}. Несмотря на то, что алгоритм
Дейкстры не подходит для несвязных графов, то, что он запускается
из каждой вершины позволяет его использовать, чтобы посчитать необходимую сумм.

\paragraph{}
Чтобы найти критическое ребро сумма сетевых расстояний считается
для всех графов $G_i$ таких, что $W(e_i) \rightarrow \gamma W(e_i)$. Если сетевые суммы 
в текущем графе $G_i$ меньше (больше при $\gamma > 1$)
ранее найденной суммы, то это ребро сохраняется в качестве претендента
на критическое. В конце работы алгоритма мы получаем критическое ребро, 
сумму сетевых расстояний, соответствующую графу с обновленным весом критического ребра и новый вес данного ребра.

\section{Реализация} 
В начале выполнения программы происходит считывание графа из файла. 
Во входном файле граф описывается в формате: $u v weight$, где $u$ и $v$ - номера вершин(тип $integer$), 
$weight$ - вес ребра (тип $double$).
Затем для каждой вершины запускаем алгоритм Дейкстры. В данной рализации можно было бы использовать 
алгоритм Флойда-Уоршелла, но было принято решение реализовать алгоритм Дейкстры, так как он легче поддается распараллеливанию.

\paragraph{}
Алгоритм реализован на языке C++. Основной процедурой является
алгоритм Дейкстры\cite{dijkstra}.

\subsection{Содержание исходного кода}

@m

\subsection{Исходный код}
\paragraph{}
Ниже приведен код класса описывающего ребро графа.
@o src/edge.h @{
#pragma once
#include <atomic>
#include <functional>

class Edge {
public:
    @< constructors @>
    @< getters and setters @>
private:
    int left;
    int right;
    double weight;
    int id;
};

@< utilities @>

@}

\paragraph{}
Класс имеет 2 конструктора, один из которых конструктор копирования.
Конструктор принимает 2 номера соединяемых вершин типа $int$, 
вес ребра типа $double$ и уникальный $id$ типа $int$, для того чтобы 
можно было эффективно искать ребра. Эти данные сохраняются в качестве
полей объекта. 

@d constructors @{
Edge(const Edge& e) :
    left(e.left), 
    right(e.right), 
    weight(e.weight), 
    id(e.id) { }
Edge(int u, int v, double w, int id) : 
    left(u), 
    right(v), 
    weight(w), 
    id(id) { }
@}

\paragraph{}
Класс имеет геттеры для всех полей объектов и сеттер для поля $weight$, т.к.
необходимо динамически изменять вес ребер в графе, чтобы пересчитывать суммы сетевых расстояний.

@d getters... @{
const int GetLeft() const { return left; }
const int GetRight() const { return right; }
const double GetWeight() const { return weight; }
const int getId() const { return id; }
void SetWeight(double w) { weight = w; }
@}

\paragraph{}
Для удобства использования структутры были написаны оператор "()" 
обеспечивающий возможность удобного доступа к элементам и оператор сравнения.

@d utilities @{
struct EdgeHash {
    unsigned int operator()(const Edge& e) const {
        return std::hash<int>()(e.GetRight());
    }
};

bool operator==(const Edge& e, const Edge& t);
@}

@o src/edge.cpp @{
#include "edge.h"

bool operator==(const Edge& e, const Edge& t) {
    return e.GetRight() == t.GetRight();
}
@}

\paragraph{}
Код класса описывающий граф.

@o src/graph.h @{
@< includes @>
class Graph {
public:
    @< graph constructors @>
    @< graph edges @>
    @< graph open @>
    @< graph dijkstra @>
    @< graph find critical @>
private:
    @< graph inf@>
    @<graph data@>
    @<graph used@>
    @<graph distsum@>
    @<graph dijkstra2@>
    @<graph run dijkstra@>
    @<graph add edge@>
};

    @<graph operator@>
@}


\paragraph{}
Зависимости класса описывающего граф.

@d includes @{
#pragma once
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <iostream>
#include <fstream>
#include <queue>
#include <string>
#include <atomic>
#include <thread>
#include <sstream>
#include <regex>
#include "edge.h"
#include <cfloat>
@}

\paragraph{}
Конструкторы класса описывающего граф.

@d graph constru... @{
Graph();
Graph(std::string filename);
@}

\paragraph{}
Поле для хранения ребер графа. Для эффективности вычислений используется std::unordered\_map. 

@d graph edges @{
std::unordered_map <int, std::unordered_set <Edge, EdgeHash>> edges;
@}

\paragraph{}
Функция для считывания графа из файла.

@d graph open @{
void open(std::string filename);
@}

\paragraph{}
Функция,запускающая алгоритм Дейкстры в разных потоках.
@d graph dijkstra @{
double RunDijkstraAsync();    
@}

\paragraph{}
Основная функция, осуществляющая поиск критического ребра в исходном графе. В качестве аргумента принимает значение $\gamma$
из исходной задачи.
@d graph find critical @{
void FindCriticalEdge(double k);
@}


\paragraph{}
Константа имтирующая бесконечность в поиске путей алгоритмом Дейкстры.
@d graph inf @{
static const long long inf = std::numeric_limits<long long>::max();
@}

\paragraph{}
Поля необходимые для хранения перенумерованных вершин графа.

@d graph data @{
std::unordered_map<int, int> coord;
std::unordered_map<int, int> coord_to_vertecies;
@}

\paragraph{}
Структура данных, сохраняющая индексы использованных ребер. 
@d graph used @{
std::unordered_set<int> usedEdges;
@}

\paragraph{}
Массив необходимый для хранения сетевых расстояний, получаемых алгоритмом Дейкстры, запущенным в разных потоках. 
@d graph distsum @{
std::vector<double> distSum;
@}

\paragraph{}
Функция реализующая алгоритм Дейкстры. Принимает на вход номер вершины.
@d graph dijkstra2 @{
double Dijkstra(int v);
@}

\paragraph{}
Функция запускающая алгоритм дейкстры $Dijkstra(int v)$.
@d graph run dijkstra @{
void RunDijkstraThread(int from, int len);
@}

\paragraph{}
Функция, добавляющая ребро в граф. Принимает на вход номера вершин (u, v), вес, индекс нового ребра.
@d graph add edge @{
void AddEdge(int u, int v, double weight, int id);   
@}

\paragraph{}
Для удобства сравнения объектов класса перегружен слудющий оператор.
@d graph operator @{
class Compare {
public:
    bool operator()(std::pair<double, int> a, std::pair<double, int> b) {
        return a.first > b.first;
    }
};
@}

Ниже приведена реализация функций описанного выше класса в $graph.h$

@O src/graph.cpp 
@{
#include <vector>
#include "graph.h"

using namespace std;

//Public methods
@<gimpl constructors@>
@<gimpl open@>
@<gimpl dijkstra@>
@<gimpl critical search @>
@<gimpl Dijkstra algorythm @>
@<gimpl run Dijkstra@>
@<gimpl add edge @>
@}

\paragraph{}
Имплементация конструкторов графа. Перегруженный конструктор принимает на вход имя файла, содержащего
данные об исходном графе.

@d gimpl constructors @{
Graph::Graph() { }

Graph::Graph(std::string filename) {
    open(filename);
} 
@}


\paragraph{}
Функция считывания файла с исходным графом.

@d gimpl open @{
void Graph::open(std::string filename) {
    if (filename != "") {
        std::ifstream in(filename);
        int index, vertex;
        double weight;
        int n = 0;
        int id = 0;
        @<gimpl open while@>

        @<gimpl open for@>
    }
    distSum.resize(edges.size(), 0);
} 
@}

\paragraph{}
Цикл, предназначенный для считывния ребер графа и перенумировывания вершин графа, 
с целью иметь возможность заменить в основной структуре, содержащей граф std::unordered\_map на std::vector
и не столкнуться с ограничениями, связанными с размером памяти. Заменить std::unordered\_map на std::vector может
потребоваться в случае, если мы хотим добиться большей производительности, так как стандартный шаблонный класс
std::vector более оптимизирован для работы с различными уровнями кэша процессора.

@d gimpl open while @{
while (!in.eof()) {
          int v;
          int u;
          in >> index >> vertex >> weight;

          @<gimpl open if@>

          
          AddEdge(u, v, weight, id++);
          AddEdge(v, u, weight, id++);
} 
@}

\paragraph{}
Проверка на то, не встречалась ли нам эта вершина ранее в процессе считывания исходного графа.
@d gimpl open if @{
if (coord.find(index) != coord.end()) {
    v = coord[index];
} 
else {
    v = n++;
    coord.insert(std::make_pair(index, v));
}
if (coord.find(vertex) != coord.end()) {
    u = coord[vertex];
}
else {
    u = n++;
    coord.insert(std::make_pair(vertex, u));
}
@}

\paragraph{}
Цикл, предназначенный для сохранения графа в памяти виде списка смежности.

@d gimpl open for @{
for (auto &i : coord)
    coord_to_vertecies[i.second] = i.first;
@}

\paragraph{}
Функция, создающая отдельные потоки, в которых мы будем запускать алгоритм Дейкстры.

@d gimpl dijkstra @{
double Graph::RunDijkstraAsync() {
    @<gimpl run threadscount@>

    @<gimpl run threads@>

    int batch = edges.size() / threads_count;
    int remainder = edges.size() % threads_count;

    @<gimpl run cycle@>
    
    @<gimpl run sum@>

    return sum;
} 
@}

\paragraph{}
Количество потоков, в которых будем запускать алгоритм Дейкстры.
@d gimpl run threadscount @{
int threads_count = 8;
@}

\paragraph{}
Массив для хранения указателей на потоки.
@d gimpl run threads @{
std::vector<thread> threads;
@}

\paragraph{}
Цикл, создающий отдельные потоки, указатели на которые сохраняются в массиве.
@d gimpl run cycle @{
for (int i = 0; i < threads_count - 1; ++i)
        threads.push_back(thread(
          &Graph::RunDijkstraThread, 
          this, 
          i * batch, 
          batch
        ));
    threads.push_back(thread(
        &Graph::RunDijkstraThread, 
        this, 
        (threads_count - 1) * batch, 
        batch + remainder
      ));

  for (int i = 0; i < threads_count; ++i)
        threads[i].join(); 
@}

\paragraph{}
Суммируем все полученные сетевые расстояния
@d gimpl run sum @{
double sum = 0;
for (auto &i : distSum)
    sum += i;
@}


\paragraph{}
Основная функция программы, которая выполняет поиск критического ребра

@d gimpl critical search @{
void Graph::FindCriticalEdge(double k) {

    @<gimpl critical init@>

    if (k >= 0.0 && k <= 1.0) {
      @< gimpl critical min@>
    }
    else if (k >= 1.0) {
      @< gimpl critical max@>  
    }

    @<gimpl critical print@>
}
@}

\paragraph{}
Инициализация необходимых перменных.
@d gimpl critical init @{
int criticalLeft = -1;
int criticalRight = -1;
int size = edges.size();
int count = 0;
double distances = 0; 
@}

\paragraph{}
Вывод ответа.
@d gimpl critical print @{
if (criticalLeft != -1 && criticalRight != -1) {
    auto edge = edges[criticalLeft].find(Edge(0, criticalRight, 0, 0));
    cout << "New distances: " << distances << ". With edge between ";
    cout << coord_to_vertecies[edge->GetLeft()] 
      << " and " 
      << coord_to_vertecies[edge->GetRight()] 
      << ". Weight: " 
      << edge->GetWeight() << endl;
} 
@}

\paragraph{}
Код отвечающий за поиск критического ребра в случае, при $\gamma \leq 1$
@d gimpl critical min @{
double min = DBL_MAX;
for (auto &v : edges) {
    cout << count++ << " out of " << size << flush;
    for (auto &edge : v.second) {
        if (usedEdges.find(edge.getId()) == usedEdges.end()) {
            double PrevWeight = edge.GetWeight();
            auto it = edges[edge.GetRight()]
              .find(Edge(0, edge.GetLeft(), 0, 0));
            int invId = it->getId();
            usedEdges.insert(invId);
            edges[edge.GetRight()].erase(it);
            edges[edge.GetRight()]
              .insert(Edge(
                edge.GetRight(), 
                edge.GetLeft(), 
                PrevWeight*k, 
                invId));
            ((Edge&)edge).SetWeight(PrevWeight*k);
            double sum = RunDijkstraAsync();
            if (min > sum) {
                min = sum;
                criticalLeft = edge.GetLeft();
                criticalRight = edge.GetRight();
            }
            ((Edge&)edge).SetWeight(PrevWeight);
            edges[edge.GetRight()]
              .erase(Edge(0, edge.GetLeft(), 0, 0));
            edges[edge.GetRight()]
              .insert(Edge(
                edge.GetRight(), 
                edge.GetLeft(), 
                PrevWeight, 
                invId));
        }
    }
    cout << "\r";
}
cout << endl;
distances = min;
@}

\paragraph{}
Код отвечающий за поиск критического ребра при $\gamma > 1$ 

@D gimpl critical max @{
double max = 0;
for (auto &v : edges) {
    cout << count++ << " out of " << size << flush;
    for (auto &edge : v.second) {
        if (usedEdges.find(edge.getId()) == usedEdges.end()) {
            double PrevWeight = edge.GetWeight();
            auto it = edges[edge.GetRight()]
              .find(Edge(0, edge.GetLeft(), 0, 0));
            int invId = it->getId();
            usedEdges.insert(invId);
            edges[edge.GetRight()].erase(it);
            edges[edge.GetRight()]
              .insert(Edge(
                edge.GetRight(), 
                edge.GetLeft(), 
                PrevWeight*k, 
                invId));
            ((Edge&)edge).SetWeight(PrevWeight*k);
            double sum = RunDijkstraAsync();
            if (max < sum) {
                max = sum;
                criticalLeft = edge.GetLeft();
                criticalRight = edge.GetRight();
            }
            ((Edge&)edge).SetWeight(PrevWeight);
            edges[edge.GetRight()]
              .erase(Edge(0, edge.GetLeft(), 0, 0));
            edges[edge.GetRight()]
              .insert(Edge(
                edge.GetRight(), 
                edge.GetLeft(), 
                PrevWeight, 
                invId));
        }
    }
    cout << "\r";
}
cout << endl;
distances = max;
@}


\paragraph{}
Реализация алгоритма Дейкстры \cite{Dijkstra}
@d gimpl Dijkstra... @{
double Graph::Dijkstra(int v) {

    @< gimpl djk vars@>

    
    @< gimpl djk while@>


    @< gimpl djk sum@>
    return sum;
}
@}

\paragraph{}
Инициализация массивов для хранения дистанций и посещенных вершин.
В очереди с приоритетами $q$, хранится пара: вес, ребро.
@d gimpl djk vars @{
std::vector<double> dist(edges.size(), inf);
std::vector<bool> visited(edges.size(), false);

dist[v] = 0.0;
std::priority_queue<
  std::pair<double, int>, 
  std::vector<std::pair<double, int>>, 
  Compare> q;
visited[v] = true;
q.push(std::make_pair(dist[v], v));
@}

\paragraph{}
Функция, которая запускает алгоритм Дейкстры на указанных вершинах.
@d gimpl run Dijkstra @{
void Graph::RunDijkstraThread(int from, int len) {
    for (int i = from; i < from + len; ++i)
        distSum[i] = Dijkstra(i);
}
@}

\paragraph{}
Фукнция добавляющая ребро в граф.
@d gimpl add edge @{
void Graph::AddEdge(int index, int vertex, double weight, int id) {
    edges[index].insert(Edge(index, vertex, weight, id));
}
@}

\paragraph{}
Основной цикл работы алгоритма Дейкстры.
@d gimpl djk while @{
while (!q.empty()) {
        std::pair<double, int> from = q.top(); q.pop();
        for (auto& edge : edges[from.second]) {
            int to = edge.GetRight();
            if (!visited[to]) {
                double tmp = from.first + edge.GetWeight();
                if (dist[to] > tmp) {
                    dist[to] = tmp;
                    visited[to] = true;
                    q.push(std::make_pair(tmp, to));
                }
            }
        }
    } 
@}

\paragraph{}
Подсчет суммы сетевых расстояний.
@d gimpl djk sum @{
double sum = 0;
    for (auto &i : dist)
        if (i != inf)
            sum += i; 
@}

\paragraph{}
Ниже приведен исходный код входной точки программы. 
Имя файла содержащего данные о графе и значение 
$\gamma$ передаются в качестве параметров интерфейса командной строки.

@o src/main.cpp @{
#include "graph.h"

using namespace std;

int main(int argc, char* argv[]) {
    cout << "Reading graph" << endl;
    Graph graph;
    if (argc > 1)
        graph.open(argv[1]);
    else
        graph.open("little.dat");
    cout << "Read graph successfully" << endl;
    cout << "Counting original distances" << endl;
    cout << "Original distances: " << graph.RunDijkstraAsync() << endl;
    double k = 0.5;
    if (argc > 2)
        k = atof(argv[2]);
    cout << "Searching critical edge" << endl;
    graph.FindCriticalEdge(k);
    return 0;
}
@}


\paragraph{Оценка вычислительной сложности алгоритма.}

На данном графе $G = (V, E, W)$, алгоритм Дейкстры имеет вычислительную  сложность $O(V \log V + E) $.
В нашем алгоритме на каждое изменение веса ребра, запускается алгоритм Дейкстры из каждой вершины. В результате получаем 
вычислительную сложность алгоритма $O((V \log V + E)VE)$

\paragraph{Оптимизации:}
Чтобы ускорить исполнение программы, из каждой
вершины алгоритм Дейкстры запускается в отдельном
потоке. Для экономии памяти граф хранится в
виде списка смежности.

\section{Результаты}

\subsection{Минимальный граф}

\paragraph{}

Рассмотрим простой пример работы алгоритма на графе с 6-ю вершинами и 5-ю ребрами
представленного на рис.~\ref{fig:min_graph_4} В данном случае мы будем минимизировать сумму сетевых расстояний и
примем $\gamma = 0.5$.

\begin{figure}[h]
    \centering
    \includegraphics[scale=0.7]{min_graph_4.png}
    \caption{Минимальный граф с критическим ребром между 3-й и 4-й вершинами.}
    \label{fig:min_graph_4}
\end{figure}

Из каждой вершины исходного графа запустим алгоритм Дейкстры.
На каждом шаге работы алгоритма изменяем вес текущего ребра $W(e_i) \rightarrow \gamma W(e_i)$
Затем для данной итерации пересчитаем сумму сетевых расстояний, и, если она меньше текущей минимальной суммы,
то обновим минимум. Так в процессе работы алгоритма мы переберем все ребра и в качестве критического выберем 
ребро с соответствующей минимальной суммой сетевых расстояний. 

\paragraph{}

На каждой итерации работы алгоритма Дейкстры получим сумму сетевых расстояний для исходного графа:
Соответственно для каждой веришны имеем следующую сумму сетевых расстояний:

\paragraph{}

Для первой веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от первой вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
1 - 3 : weight sum = 1 \\
1 - 3 - 4 : weight sum = 5 \\
1 - 3 - 4 - 5 : weight sum = 6 \\
1 - 3 - 4 - 6 : weight sum = 6 \\
1 - 2 : weight sum = 2
\end{gather}
$Sum_1 = 1 + 5 + 6 + 6 + 2 = 20$

\paragraph{}

Для второй веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от второй вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
2 - 3 : weight sum = 1 \\
2 - 3 - 4 : weight sum = 5 \\
2 - 3 - 4 - 5 : weight sum = 6 \\
2 - 3 - 4 - 6 : weight sum = 6 \\
2 - 1 : weight sum = 2
\end{gather}
$Sum_2 = 1 + 5 + 6 + 6 + 2 = 20$

\paragraph{}

Для третьей веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от третьей вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
3 - 1 : weight sum = 1 \\
3 - 2 : weight sum = 1 \\
3 - 4 : weight sum = 4 \\
3 - 4 - 5 : weight sum = 5 \\
3 - 4 - 6 : weight sum = 5 \\
\end{gather}
$Sum_3 = 1 + 1 + 4 + 5 + 5 = 16$

\paragraph{}

Для четвертой веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от четвертой вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
4 - 5 : weight sum = 1 \\
4 - 6 : weight sum = 1 \\
4 - 3 : weight sum = 4 \\
4 - 3 - 1 : weight sum = 5 \\
4 - 3 - 2 : weight sum = 5 \\
\end{gather}
$Sum_4 = 1 + 1 + 4 + 5 + 5 = 16$

\paragraph{}

Для пятой веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от пятой вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
5 - 3 : weight sum = 1 \\
5 - 4 - 3 : weight sum = 5 \\
5 - 4 - 3 - 1 : weight sum = 6 \\
5 - 4 - 3 - 2 : weight sum = 6 \\
5 - 6 : weight sum = 2
\end{gather}
$Sum_5 = 1 + 5 + 6 + 6 + 2 = 20$

\paragraph{}

Для шестой веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от шестой вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
5 - 3 : weight sum = 1 \\
5 - 4 - 3 : weight sum = 5 \\
5 - 4 - 3 - 1 : weight sum = 6 \\
5 - 4 - 3 - 2 : weight sum = 6 \\
5 - 6 : weight sum = 2
\end{gather}
$Sum_6 = 1 + 5 + 6 + 6 + 2 = 20$

\paragraph{}

Таким образом получим сумму сетевых расстояний в исхожном графе $S = 20 + 20 + 16 + 16 + 20 + 20 = 112$.

\begin{figure}[h]
    \centering
    \includegraphics[scale=0.7]{min_graph_2.png}
    \caption{Если уменьшить вес ребра между 3-й и 4-й вершинами сумма сетевых расстояний минимизируется}
    \label{fig:min_graph_2}
\end{figure}

В данном граффе критическим является ребро между
3й и 4й вершинами и умножение его веса на
$\gamma < 1$ приводит к минимизации суммы сетевых расстояний.
При $\gamma = 0.5$ алгоритм меняет вес критического ребра на $0.5 \cdot w_i$ (рис.~\ref{fig:min_graph_2}).

\paragraph{}

На каждой итерации работы алгоритма Дейкстры получим сумму сетевых расстояний для обновленного графа:
Соответственно для каждой веришны имеем следующую сумму сетевых расстояний:

\paragraph{}

Для первой веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от первой вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
1 - 3 : weight sum = 1 \\
1 - 3 - 4 : weight sum = 3 \\
1 - 3 - 4 - 5 : weight sum = 4 \\
1 - 3 - 4 - 6 : weight sum = 4 \\
1 - 2 : weight sum = 2
\end{gather}
$Sum_1 = 1 + 3 + 4 + 4 + 2 = 14$

\paragraph{}

Для второй веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от второй вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
2 - 3 : weight sum = 1 \\
2 - 3 - 4 : weight sum = 3 \\
2 - 3 - 4 - 5 : weight sum = 4 \\
2 - 3 - 4 - 6 : weight sum = 4 \\
2 - 1 : weight sum = 2
\end{gather}
$Sum_2 = 1 + 3 + 4 + 4 + 2 = 14$

\paragraph{}

Для третьей веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от третьей вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
3 - 1 : weight sum = 1 \\
3 - 2 : weight sum = 1 \\
3 - 4 : weight sum = 2 \\
3 - 4 - 5 : weight sum = 3 \\
3 - 4 - 6 : weight sum = 3 \\
\end{gather}
$Sum_3 = 1 + 1 + 2 + 3 + 3 = 10$

\paragraph{}

Для четвертой веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от четвертой вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
4 - 5 : weight sum = 1 \\
4 - 6 : weight sum = 1 \\
4 - 3 : weight sum = 3 \\
4 - 3 - 1 : weight sum = 4 \\
4 - 3 - 2 : weight sum = 4 \\
\end{gather}
$Sum_4 = 1 + 1 + 2 + 3 + 3 = 10$

\paragraph{}

Для пятой веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от пятой вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
5 - 3 : weight sum = 1 \\
5 - 4 - 3 : weight sum = 3 \\
5 - 4 - 3 - 1 : weight sum = 4 \\
5 - 4 - 3 - 2 : weight sum = 4 \\
5 - 6 : weight sum = 2
\end{gather}
$Sum_5 = 1 + 3 + 4 + 4 + 2 = 14$

\paragraph{}

Для шестой веришны сумма сетевых расстояний будет состоять из сумм представленных ниже 
путей от шестой вершины графа к остальным.
Далее через тире обозначенны вершины графа и сумма весов всех ребер в данном пути:
\begin{gather}
5 - 3 : weight sum = 1 \\
5 - 4 - 3 : weight sum = 3 \\
5 - 4 - 3 - 1 : weight sum = 4 \\
5 - 4 - 3 - 2 : weight sum = 4 \\
5 - 6 : weight sum = 2
\end{gather}
$Sum_6 = 1 + 3 + 4 + 4 + 2 = 14$

\paragraph{}

Для обновленного графа, в котором мы заменили вес критического ребра на 2 получим сумму сетевых расстояний:
В таком случае сумма сетевых расстояний для обновленного графа будет равнятся $S^* = 14 + 14 + 10 + 10 + 14 + 14 = 76$.

\paragraph{При $\gamma = 2 > 1$.}
Алгоритм меняет вес критического ребра на $2 \cdot w_i = 8$ (рис.~\ref{fig:min_graph_8}).

\begin{figure}[h]
    \centeringp
    \includegraphics[scale=0.7]{min_graph_8.png}
    \caption{Если увеличить вес ребра между 3-й и 4-й вершинами сумма сетевых расстояний максимизируется}
    \label{fig:min_graph_8}
\end{figure}

В обновленном графе, с весом критического ребра = 8 мы получим следующую
сумму сетевых расстояний:

Что дает сумму сетевых расстояний равную $S^* = 184$

\subsection{Граф малого размера (20 вершин)}

\paragraph{}
При запуске на графе малого размера (рис~\ref{fig:small}) 
алгоритм корректно определил критическое ребро.
Этому ребру был намеренно предан большой вес для удобства тестирования.
При $\gamma = 0.5$ и при $\gamma = 1.5$ алгоритм определял одно
и то же ребро между 19-й и 20-й вершинами (отмечено красным),
однако изменение веса этого ребра приводило к разным итоговым
суммам сетевых расстояний, а именно к $12542$ и $14264$ соотвественно.

\begin{figure}[h]
    \centering
    \includegraphics[scale=0.3]{small.png}
    \caption{Малый граф с 20-ю вершинами. Критическое ребро отмечено красным.}
    \label{fig:small}
\end{figure}

\subsection{Граф среднего размера (~1500 вершин)}

\paragraph{}
В качестве графа среднего размера был взят сокращеное представление 
транспортной сети города Владивостока на 2009-й год (рис.~\ref{fig:vlad_2009}). 
В данном представлении 1542 вершины и 1653 ребра.

\begin{figure}[h]
    \centering
    \includegraphics[scale=0.3]{vlad_2009.png}
    \caption{Сокращенная транспортная сеть города Владивостока на 2009-й год.}
    \label{fig:vlad_2009}
\end{figure}

\paragraph{}
Алгоритм был запущен с значениями $\gamma = 0.5$ и $\gamma = 24.5$ и определил 
критическое ребра между 175й и 176й вершиной и между 159-й и 160-й вершинами соответвенно.
На рис~\ref{fig:vlad_2009_min} изображен граф с отмеченными найденными критическими ребрами.
Исходная сумма сетевых расстояний составляет $2.64371\cdot10^{10}$.
При $\gamma = 0.5$ сумма сетевых расстояний составила $2.60341\cdot10^{10}$. 
При $\gamma = 24.5$ сумма сетевых расстояний составила $3.84203\cdot10^{10}$.

\paragraph{}

Без флагов оптимизации нашей программе потребовалось 7 минут 39 секнуд реального времени.
С флагом оптимизации -O2 нашей программе потребовалось 45 секунд реального времени.
Программа использовала 16 потоков. Время измерялось утилитой time.

\begin{figure}[h]
    \centering
    \includegraphics[scale=0.3]{vlad_2009_min.png}
    \caption{Сокращенная транспортная сеть города Владивостока на 2009-й год.}
    \label{fig:vlad_2009_min}
\end{figure}

\section{Заключение}

\paragraph{}
В результате проделанной работы был разработан алгоритм поиска критического ребра в графе.
Алгоритм был протестирован на графах различных размерностей и корректно решил поставленную задачу.

\newpage

\begin{thebibliography}{9}
\bibitem{dijkstra}
Dijkstra E. W. \textit{A note on two problems in connection with graphs} //
\textit{Numer. Math} — Springer Science+Business Media, 1959.
— Vol. 1, Iss. 1. — P. 269–271.
\end{thebibliography}

Для визуализации графов использовалась программа с открытым исходным кодом $gephi$.
\url{https://github.com/gephi/gephi}

\end{document}
