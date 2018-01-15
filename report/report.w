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
  Кирпа Вадим
  \and
  Махлярчук Андрей
  \and
  Утин Никита
  \and
  Березкин Аркадий
%  \and
%  Блинов Игорь
}

\maketitle
\thispagestyle{empty}
\newpage

\section{Постановка задачи:}

\paragraph{}
Для графа $G = (V, E, W)$ с множеством вершин $V$,
множеством ребер $W: E \rightarrow \mathbb{R}_+$
найти ребро $e^*$, такое, что при замене
$W(e^*) \rightarrow \gamma W(e^*)$ сумма сетевых 
расстояний между всеми узлами минимизируется
(при $\gamma < 1$) или максимизируется (при $\gamma > 1$).
Расчеты привести для графа Владивостока-2012.

\section{Алгоритм}

\paragraph{}
Для нахождения суммы сетевых расстояний в графе использовался
алгоритм Дейкстры\cite{dijkstra}.

\paragraph{}
Чтобы найти критическое ребро сумма сетевых расстояний считается
для всех подграфов $G_i$, где $W(e_i) \rightarrow \gamma W(e_i)$. Если сумма сетевых
ребер в текущем подграфе $G_i$ меньше (больше при $\gamma > 1$)
ранее найденой суммы, то это ребро сохраняется в качестве претендента
на критическое. В конце работы алгоритма мы получаем критическое ребро, 
сумму сетевых расстояний, соответствующую графу с обновленным весом критического ребра и новый вес данного ребра.

\section{Реализация} 
В начале выполнения программы происходит считывание графа из файла. 
Затем для каждой вершины запускаем алгоритм Дейкстры. В данной рализации можно было бы использовать 
алгоритм Флойда-Уоршелла, но было принято решение реализовать алгоритм Дейкстры, так как он легче поддается распараллеливанию.

\paragraph{}
Алгоритм реализован на языке C++. Основной процедурой является
алгоритм Дейкстры\cite{dijkstra}.


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
Конструктор мринимает 2 номера соеденяемых вершин типа $int$, 
вес ребра типа $double$ и уникальный $id$ типа, для того чтобы 
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
Для удобства использования струкрутры были написаны операторы.

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
Поле для хранения ребер графа. Для эффективности вычислений используется std::unordered_map. 

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
Функция, добавляющая ребро в граф. Принимает на вход номера вершин, вес, индекс нового ребра.
@d graph add edge @{
void AddEdge(int index, int vertex, double weight, int id);   
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

Ниже приведена реализация фукнций описанного выше класса в $graph.h$

@O src/graph.cpp 
@{
#include <vector>
#include "graph.h"

using namespace std;

//Public methods

Graph::Graph() { }

Graph::Graph(std::string filename) {
    open(filename);
}

void Graph::open(std::string filename) {
    if (filename != "") {
        std::ifstream in(filename);
        int index, vertex;
        double weight;
        int n = 0;
        int id = 0;
        while (!in.eof()) {
            int v;
            int u;
            in >> index >> vertex >> weight;
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
            AddEdge(u, v, weight, id++);
            AddEdge(v, u, weight, id++);
        }

        for (auto &i : coord)
            coord_to_vertecies[i.second] = i.first;
    }
    distSum.resize(edges.size(), 0);
}

double Graph::RunDijkstraAsync() {
    int threads_count = 8; //todo replace

    std::vector<thread> threads;
    int batch = edges.size() / threads_count;
    int remainder = edges.size() % threads_count;
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

    double sum = 0;
    for (auto &i : distSum)
        sum += i;       
    return sum;
}

void Graph::FindCriticalEdge(double k) {
    int criticalLeft = -1;
    int criticalRight = -1;
    int size = edges.size();
    int count = 0;
    double distances = 0;
    if (k >= 0.0 && k <= 1.0) {
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
    }
    else if (k >= 1.0) {
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
        distances = max;
    }
    else {

    }
    if (criticalLeft != -1 && criticalRight != -1) {
        auto edge = edges[criticalLeft].find(Edge(0, criticalRight, 0, 0));
        cout << "New distances: " << distances << ". With edge between ";
        cout << coord_to_vertecies[edge->GetLeft()] 
          << " and " 
          << coord_to_vertecies[edge->GetRight()] 
          << ". Weight: " 
          << edge->GetWeight() << endl;
    }
}


double Graph::Dijkstra(int v) {
    std::vector<double> dist(edges.size(), inf);
    std::vector<bool> visited(edges.size(), false);
    dist[v] = 0.0;
    std::priority_queue<
      std::pair<double, int>, 
      std::vector<std::pair<double, int>>, 
      Compare> q;
    visited[v] = true;
    q.push(std::make_pair(dist[v], v));

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
    double sum = 0;
    for (auto &i : dist)
        if (i != inf)
            sum += i;
    return sum;
}

void Graph::RunDijkstraThread(int from, int len) {
    for (int i = from; i < from + len; ++i)
        distSum[i] = Dijkstra(i);
}

void Graph::AddEdge(int index, int vertex, double weight, int id) {
    edges[index].insert(Edge(index, vertex, weight, id));
}

@}

\paragraph{}
Ниже приведен исходный код входной точки пограммы. 
Имя файла и значение $\gamma$ передаются в качестве
параметров интерфейса командной строки.

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
    double k = 1.5;
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
потоке. Т.к. граф разрежен в памяти он хранится в
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
На каждой итерации работы алгоритма Дейкстры получим сумму сетевых расстояний для исходного графа:

\begin{gather}
1 : 1 + 5 + 6 + 6 + 2 \\
2 : 1 + 5 + 6 + 6 + 2 \\
3 : 1 + 1 + 4 + 5 + 5 \\
4 : 1 + 1 + 4 + 5 + 5 \\
5 : 1 + 5 + 6 + 6 + 2 \\
6 : 1 + 5 + 6 + 6 + 2
\end{gather}

Сумма сетевых расстояний в исходном графе $S = 112$. 

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
Для обновленного графа, в котором мы заменили вес критического ребра на 2 получим сумму сетевых расстояний:

\begin{gather}
1 : 1 + 3 + 4 + 4 + 2 \\
2 : 1 + 3 + 4 + 4 + 2 \\
3 : 1 + 1 + 2 + 3 + 3 \\
4 : 1 + 1 + 2 + 3 + 3 \\
5 : 1 + 3 + 4 + 4 + 2 \\
6 : 1 + 3 + 4 + 4 + 2
\end{gather}

В таком случае сумма сетевых расстояний для обновленного графа будет равнятся $S^* = 76$.

\paragraph{При $\gamma = 2 > 1$.}
Алгоритм меняет вес критического ребра на $2 \cdot w_i = 8$ (рис.~\ref{fig:min_graph_8}).

\begin{figure}[h]
    \centering
    \includegraphics[scale=0.7]{min_graph_8.png}
    \caption{Если увеличить вес ребра между 3-й и 4-й вершинами сумма сетевых расстояний максимизируется}
    \label{fig:min_graph_8}
\end{figure}

В обновленном графе, с весом критического ребра = 8 мы получим следующую
сумму сетевых расстояний:

\begin{gather}
1 : 1 + 9 + 10 + 10 + 2 \\
2 : 1 + 9 + 10 + 10 + 2 \\
3 : 1 + 1 + 8  + 9  + 9 \\
4 : 1 + 1 + 8  + 9  + 9 \\
5 : 1 + 9 + 10 + 10 + 2 \\
6 : 1 + 9 + 10 + 10 + 2
\end{gather}

Что дает сумму сетевых расстояний равную $S^* = 184$

\subsection{Граф малого размера (20 вершин)}

\paragraph{}
При запуске на графе малого размера (рис~\ref{fig:small}) 
алгоритм корректно определил критическое ребро.
Этому ребру был намеренно предан большой вес для удобства тестирования.
При $k = 0.5$ и при $k = 1.5$ алгоритм определял одно
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
транспортной сети города Владивостока на 2009-й годi (рис.~\ref{vlad_2009}). 
В данном представлении 1542 вершины и 1653 ребра.

\begin{figure}[h]
    \centering
    \includegraphics[scale=0.3]{vlad_2009.png}
    \caption{Сокращенная транспортная сеть города Владивостока на 2009-й год.}
    \label{fig:vlad_2009}
\end{figure}

\paragraph{}
Алгоритм был запущен с значениями $k = 0.5$ и $k = 1.5$ и определил различные
критические ребра. Ребро между

% New distances: 2.60341e+10. With edge between 175 and 176. Weight: 1008.3; k = 0.5
% New distances: 2.67274e+10. With edge between 175 and 176. Weight: 1008.3; k = 1.5

\begin{figure}[h]
    \centering
    \includegraphics[scale=0.3]{vlad_2009_min_max.png}
    \caption{Сокращенная транспортная сеть города Владивостока на 2009-й год.}
    \label{fig:vlad_2009_min_max}
\end{figure}

\section{Заключение}
Окончательное и обжалованию не подлежит

\newpage

\begin{thebibliography}{9}
\bibitem{dijkstra}
Dijkstra E. W. \textit{A note on two problems in connexion with graphs} //
\textit{Numer. Math} — Springer Science+Business Media, 1959.
— Vol. 1, Iss. 1. — P. 269–271.
\end{thebibliography}

\end{document}
