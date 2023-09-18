# Grafos
## Definiciones básicas
- Los **grafos** son un conjunto de vertices y ejes G = (V, E)
- si no aclara no tiene dirección
- No hay selfloops ni varios ejes para un mismo par de nodos (multigrafos)
- ### Tipos de grafos
  - **Digrafo**: grafo con ejes con dirección (a lo sumo puede tener 2 ejes por par de nodos, uno para la ida y otro para la vuelta)
  - Dos grafos son isomorfismo si son iguales salvo por el nombre de los nodos (los rotas y quedan igual en distribución)
  - **Grafos conectados**: (para grafos sin dirección), hay un camino entre cualquier par de nodos
  - **Grafos fuertemente conectados**: (para grafos con dirección), hay una ruta entre cualquier par de nodos
  - **Grafos completos**: (para grafos sin dirección), todos los nodos estan conectados con todos
  - **Grafo complemento**: Es el grafo con el mismo conjunto de vertices pero solo tiene las aristas que NO estan en G.
  - **Grafo conexo**: si existe un camino para todo par de vertices
  - **Grafos bipartitos**:
  - **subgrafos**:
  ![Alt text](./img/definiciones_subgrafos.png)
  
  - **Componente conexa**: es un subgrafo conexo maximal (no está incluído estrictamente en otro grafo).
  ![Alt text](./img/ejemplo_componentes_conexas.png)

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

## Teorema 1: suma de grados
$\sum _{v \in V}{d(v)}$ = $2m$

siendo m la cantidad de aristas del grafo

***colorario*** : la cantidad de nodos con grado impar es par. Esto debido a que la suma de todas las conexiones da un numero impar. Si sumamos 2 pares da par, si sumamos 2 impares de par, por lo que para preservar la paridad tiene que haber una cantidad par de nodos con grado impar. 




