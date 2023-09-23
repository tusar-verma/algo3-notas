# Grafos
## Definiciones básicas
- Los **grafos** son un conjunto de vertices y ejes G = (V, E)
- si no aclara no tiene dirección
- No hay selfloops ni varios ejes para un mismo par de nodos (multigrafos)
- ### Tipos de grafos
  - **Digrafo**: grafo con ejes con dirección (a lo sumo puede tener 2 ejes por par de nodos, uno para la ida y otro para la vuelta)
    - $d_{in}(v)$ : grado de entrada de un nodo
    - $d_{out}(v)$ : grado de salida de un nodo
    - las aristas E ahora son pares ordenados donde, sea e=(u, v), e se lo llama arco, u cola (desde donde viene) y v cabeza (hacia donde va)
  - Dos grafos son isomorfismo si son iguales salvo por el nombre de los nodos (los rotas y quedan igual en distribución)
  - **Grafos conectados**: (para grafos sin dirección), hay un camino entre cualquier par de nodos
  - **Grafos fuertemente conectados**: (para grafos con dirección), hay una ruta entre cualquier par de nodos
  - **Grafos completos**: (para grafos sin dirección), todos los nodos estan conectados con todos
  - **Grafo complemento**: Es el grafo con el mismo conjunto de vertices pero solo tiene las aristas que NO estan en G.
  - **Grafo conexo**: si existe un camino para todo par de vertices
  - **Grafo fuertemente conexo**: (directed graphs) si existe un camino orientado entre todo par de vertices.
  - **Grafos bipartitos**: es un grafo cuyos vértices se pueden separar en dos conjuntos disjuntos, de manera que las aristas no pueden relacionar vértices de un mismo conjunto.1
  ![Alt text](./img/grafos_bipartitos.png)
  - **Grafos bipartitos completos**: es un grafo bipartito en que todos los vértices de uno de los subconjuntos están relacionados con los del otro subconjunto
    - ### teoremas

  - **subgrafos**:
  ![Alt text](./img/definiciones_subgrafos.png)
  
  - **Componente conexa**: es un subgrafo conexo maximal (no está incluído estrictamente en otro grafo).
  ![Alt text](./img/ejemplo_componentes_conexas.png)

  - **Arboles**: 
    - Grafo conectado y aciclico
    - si saco una arista, se desconecta
    - si agrego una arista se forma un ciclo

- ### caminos y recorridos    
  - **Vecindario**: N(v) es el conjunto de nodos de G adyacentes a v

  - **grado**: d(v) = |N(v)| es la cantidad de vecinos
  - **recorrido**: una secuencia de nodos conectados por aristas (pueden repetir nodos)
  - **longitud de un recorrido**: la cantidad de *aristas* que tiene
  - **Distancia** entre 2 nodos: el camino mas corto d(u,v). Si no existe es $\infin$. La distancia de un vertice con si mismo es 0.
  - **camino**: un recorrido sin nodos repetidos (tambien se llama camino simple cuando se refiere al recorrido como camino)
  - **circuito**: un recorrido que empieza y termina en el mismo nodo
  - **ciclo**: un circuito que no repite nodos (se puede decir ciclo al circuito y circuito simple al circuito)
  - No es valido un ciclo de longitud 2
  - **puente**: es una arista que al quitarla aumenta la cantidad de componentes conexas del grafo

- ### **Isomorfismo de grafos**:
  ![Alt text](./img/definicion_isomorfismo.png)
  ![Alt text](./img/propiedades_isomorfismo.png)
## Teorema 1: suma de grados
$\sum _{v \in V}{d(v)}$ = $2m$

siendo m la cantidad de aristas del grafo

***colorario*** : la cantidad de nodos con grado impar es par. Esto debido a que la suma de todas las conexiones da un numero impar. Si sumamos 2 pares da par, si sumamos 2 impares de par, por lo que para preservar la paridad tiene que haber una cantidad par de nodos con grado impar. 

## DFS y BFS

### DFS sin stack

```c++
vector<vector<int>> aristas = ... ;
vector<bool> visitado(n, false);

void dfs(int v) {
  visitado[v] = true;

  for (int u : aristas[v]) {
    if (!visitado[u]) {
      dfs(u)
    }
  }

}
```

### DFS con stack
```c++
vector<vector<int>> aristas = ... ;
vector<bool> visitado(n, false);

void dfs(int v) {
  stack<int> s;
  s.push(v);
  vistados[v] = true

  while (!s.empty()) {
    int v = s.top(); s.pop();

    for (auto u: aristas[v]) {
      if (!visitado[u]) {
        visitado[u] = true;
        s.push(u);
      }
    }
    
  }

}

```

### BFS con cola


```c++
vector<vector<int>> aristas = ...;
vector<bool> visitados(n, false);
vector<int> distancia(n);

void bfs(int s) {
  visitados[s] = true
  distancia[s] = 0

  queue<int> q;
  q.push(s);

  while (!q.empty()) {
    int v = q.front(); q.pop();

    for (auto u : aristas[v]) {
      if (!visitado[u]) {
        visitado[u] = true;
        distancia[u] = distancia[v] +1;
        q.push(u);
      }
    }
  }
}
```

### Problemas que se resuelven con DFS y BFS

1. **¿Es el grafo conexo?**: Recorro con DFS o BFS desde algún nodo (el 0 por ejemplo) y si el vector de visitados no tiene ningún falso, entonces sí es conexo. De lo contrario hay nodos que desde el que comenzamos no podemos llegar. 
2. **¿Cuantas componentes conexas tengo?**: Cada vez que recorro el grafo desde un vertice, me marca como visitado los que pertenecen a su componente conexa. Luego me fijo cuales no fueron marcadas, recorro desde la primera no marcada para encotnrar todos los elementos de la otra componente conexa. Repito para obtener todas las componentes conexas.
```c++
int cant_comp_conexas = 0;
for (int i = 0; i < n; i++>) { // n la cantidad de vertices del grafo 
  if (!visitado[i]) {
    cant_comp_conexas++;
    recorro_desde_el_vertice(i); // con dfs o bfs
  } 
}
```

3. **¿Hay ciclos? Y si hay, guardame alguno**: Recorro el grafo y en cada paso guardo el padre del nodo actual. Recorro los vecinos del nodo actual v. Para cada vecino u, si no está recorrida le pongo el padre y la recorro, si está recorrida y no es el padre de v, entonces se hay un ciclo desde v hasta ese u    
```c++
vector<int> padres(n, -1);
vector<int> ciclo;
vector<vector<int>> aristas = ...;
int comienzo_ciclo = -1, fin_ciclo = -1;
queue<int> q;


for (int i = 0; i < n && comienzo_ciclo == fin_ciclo == -1; i++) {
  if (padre[i] == -1) {
    dfs_ciclos(i);
  }
}

if (comienzo_ciclo >= 0) {
  int v = comienzo_ciclo;
  ciclo.push_back(v);

  while (v != fin_ciclo) {
    v = padre[v];
    ciclo.push_back(v);
  }
}



void dfs_ciclos(int s) {
  padre[s] = 0;
  q.push(s);

  while (!q.empty() && comnienzo_ciclo == fin_ciclo == -1) {
    int v = q.top(); q.pop();

    for (auto u : aristas[v]) {
      if (padre[u] == -1) {
        padre[u] = v;
        q.push(u);
      } else if (u != padre[v]) {
        //si la arista no es la que apunta al padre de v, ciclo.
        comienzo_ciclo = v;
        fin_ciclo = u;
        break;
      }
    }
  }
}
```
4. **¿Es un grafo bipartito?**: Recorro el grafo pintando de 2 colores, cuando me encuentro en el recorrido uno del mismo color que su padre, entonces no es bipartito
```c++
void dfs_bipartito(int v) {
  for (int u : aristas[v]) {
    if (color[u] == -1) { 
      // si no esta pintado pinto
      color[u] = 1 - color[v];
      dfs(u);
    } else if (color[u] == color[v]) { 
      // si ya esta pintado chequeo
      es_bipartito = false;
    }
  }
}
// llamada
es_bipartito = true;
color[0] = 0;
dfs_bipartito(0);
```
5. **Cantidad de caminos desde w a v**: Luego de hacer BFS en w, tengo computadas todas las distancias desde w hacia cualquier nodo que tenga un camino con w. Luego para buscar la cantida de caminos, cuento la cantidad de caminos que hay desde los vecinos con distancia menor a v. Cuando la distancia es 0, estoy en el nodo w y hay un solo camino.
```c++
BFS(w)
cantidad_de_caminos_hasta(v);

int cantidad_de_caminos_hasta(v) {
  if (distancia[v] == 0) return 1;
  if (memo[v] != -1) return memo[v];
  int res = 0;

  for (int vecino : aristas[v]) {
    if (distancia[vecino] + 1 == distancia[v]) {
      res += cantidad_de_caminos_hasta(vecino);
    }
  }

  memo[v] = res;
  return res;
}

```
6. **Cantidad de puentes de un grafo**: Un puento es una arista que al quitarla aumenta la cantidad de componentes conexas del grafo<br>
En el recorrido DFS puedo tener 3 estados para cada vertice: Empecé a recorrer, terminé de recorrer, no lo recorrí. Según estos estados podemos clasificar a las aristas según el arbol generado por el DFS.<br>
Dado un momento del recorrido en un vertice V, vamos a ver todos sus vecinos. Si el vecino no lo recorrí, entonces la arista es un tree-edge (pertenece al arbol DFS). Si no, y si no es el padre, entonces es un back-edge (conecta con un ancestro) y no pertenece al arbol DFS. Una back-edge no es un puente <br>
Una arista es un puente si es un tree-edge y no tiene una back-edge "que la cubra". La cantidad de back-edges que cubre a una arista v de su padre es <br><br>
$\sum_{\text{w hijo de v}}{cubren(w)}$ $- backEdgesQueTerminanEn(v) + backEdgesQueEmpiezanEn(v)$ 
<br><br>
*backEdgesQueTerminanEn(v)*: aristas que no estan en el DFS que empiezan en algun decendiente de v y que terminan conectado en v.<br>
*backEdgesQueEmpiezanEn(v)*: aristas que empiezan en v y van hacia algun ancestro de v.<br>

```c++
vector<int> memo(n, -1);
int NO_LO_VI = 0, EMPECE_A_VER = 1, TERMINE_DE_VER = 2;
vector<int> estado(n, NO_LO_VI);
vector<vector<int>> tree_edges(n), backEdgesQueTerminanEn(n), backEdgesQueEmpiezanEn(n);

void dfs_puentes(int v, int p = -1) {
  estado[v] = EMPECE_A_VER;
  for (int u : aristas[v]) {
    if (estado[u] == NO_LO_VI) {
      tree_edges[v].push_back(u);
      dfs(u, v);
    } else if (u != p) {
      backEdgesQueEmpiezanEn[v]++;
      backEdgesQueTerminanEn[u]++;
    }
  }
  estado[v] = TERMINE_DE_VER;
}

int cubren(int v, int p = -1) {
  if (memo[v] != -1) return memo[v];
  int res = 0;

  for (int hijo : tree_edges[v]) {
    if (hijo != p) {
      res += cubren(hijo, v);
    }
  }

  res -= backEdgesQueTerminanEn[v];
  res += backEdgesQueEmpiezanEn[v];
  memo[v] = res;
  return res;
}

int componentes = 0, cant_puetnes = 0;

for (int i = 0; i < n; i++) {
  if (estado[i] != NO_LO_VI) {
    dfs_puentes(i);
    componentes++;
  }
}
for (int i = 0; i < n; i++) {
  if (cubren[i] == 0){
    cant_puentes++;
  }
}
// Por cada componente conexa hay una raiz que no lo cubre nadie, pero no cuenta para aristas puente
cant_puentes -= componentes

```
7. **Topological sort**: Dado un grafo acíclico dirigido G, es una ordenación lineal de todos los nodos de G que satisface que si G contiene la arista dirigida uv entonces el nodo u aparece antes del nodo v. En grafos con ciclos no hay orden topológico. (Se ordenan los nodos en orden de precedencia)
```c++
vector<lista<int>> aristas = ...;
vector<bool> visitado(bool, false)
vector<int> vertices_ordenados;
stack<int> res;

for (int i = 0; i < n; i++) {
  if (!visitado[i]) {
    dfs_topological_sort(i);
  }
}

for(int i = 0; i < n; i++) {
  vertices_ordenados.push_back(res.pop());
}

void dfs_topological_sort(int v) {
  visitado[v] = true;

  for (auto u : aristas[v]) {
    if (!visitado[u]) {
      dfs_topological_sort(u);
    }
  }
  res.push(v);

}
```
