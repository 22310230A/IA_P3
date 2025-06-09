import networkx as nx
import matplotlib.pyplot as plt

# ===============================
# Grafo representando rutas reales
# ===============================
grafo = {
    'Guadalajara': [('Aguascalientes', 180), ('León', 210)],
    'Aguascalientes': [('Guadalajara', 180), ('San Luis', 160)],
    'León': [('Guadalajara', 210), ('Querétaro', 180)],
    'San Luis': [('Aguascalientes', 160), ('Querétaro', 220)],
    'Querétaro': [('San Luis', 220), ('León', 180), ('CDMX', 180)],
    'CDMX': [('Querétaro', 180)]
}

# ===============================
# Convertir grafo a lista de aristas únicas
# ===============================
aristas = []
agregadas = set()

for origen in grafo:
    for destino, peso in grafo[origen]:
        if (destino, origen) not in agregadas:
            aristas.append((peso, origen, destino))
            agregadas.add((origen, destino))

# ===============================
# Algoritmo de Kruskal con impresión paso a paso
# ===============================
def kruskal(aristas, modo='min'):
    print(f"\n🔍 Ejecutando Kruskal ({'mínimo' if modo == 'min' else 'máximo'} costo):")

    if modo == 'min':
        aristas = sorted(aristas)
    elif modo == 'max':
        aristas = sorted(aristas, reverse=True)

    padre = {}
    def find(nodo):
        while padre[nodo] != nodo:
            padre[nodo] = padre[padre[nodo]]
            nodo = padre[nodo]
        return nodo

    def union(nodo1, nodo2):
        raiz1 = find(nodo1)
        raiz2 = find(nodo2)
        if raiz1 != raiz2:
            padre[raiz2] = raiz1
            return True
        return False

    nodos = set()
    for _, u, v in aristas:
        nodos.add(u)
        nodos.add(v)
    for nodo in nodos:
        padre[nodo] = nodo

    resultado = []
    for peso, u, v in aristas:
        print(f"Revisando arista: {u} ↔ {v} con peso {peso}")
        if union(u, v):
            resultado.append((u, v, peso))
            print(f"   Agregada al árbol")
        else:
            print(f"    Crea ciclo, descartada")

    return resultado

# ===============================
# Graficar el árbol generado
# ===============================
def graficar_arbol(aristas_usadas, modo):
    G = nx.Graph()
    for peso, u, v in aristas:
        G.add_edge(u, v, weight=peso)

    pos = nx.spring_layout(G, seed=42)
    edge_labels = nx.get_edge_attributes(G, 'weight')

    plt.figure(figsize=(10, 6))
    nx.draw(G, pos, with_labels=True, node_size=1000, node_color='lightblue', font_size=11)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

    aristas_dibujar = [(u, v) for u, v, _ in aristas_usadas]
    nx.draw_networkx_edges(G, pos, edgelist=aristas_dibujar, width=3, edge_color='green')

    if modo == 'min':
        plt.title('Árbol de Expansión de Mínimo Costo (Kruskal)')
        plt.text(0, -1.1, 'Conexión total de ciudades con el menor tiempo total estimado.', fontsize=10, ha='center', color='gray')
    else:
        plt.title('Árbol de Expansión de Máximo Costo (Kruskal)')
        plt.text(0, -1.1, 'Conexión total de ciudades con el mayor tiempo total posible.', fontsize=10, ha='center', color='gray')

    plt.axis('off')
    plt.tight_layout()
    plt.show()

# ===============================
# Ejecutar y mostrar resultados
# ===============================
arbol_min = kruskal(aristas, modo='min')
arbol_max = kruskal(aristas, modo='max')

graficar_arbol(arbol_min, modo='min')
graficar_arbol(arbol_max, modo='max')
