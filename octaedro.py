import numpy as np
import pygame
import sys
import doctest

# --- Definições Iniciais do Objeto e Viewport ---

# Objeto: Octaedro (6 Vértices)
VERTICES_CARTESIANAS = np.array([
    [ 1,  0,  0],  # Ponta +X
    [-1,  0,  0],  # Ponta -X
    [ 0,  1,  0],  # Ponta +Y
    [ 0, -1,  0],  # Ponta -Y
    [ 0,  0,  1],  # Ponta +Z
    [ 0,  0, -1]   # Ponta -Z
]).T

# Adiciona a coordenada homogênea '1' (w=1)
VERTICES_HOMOGENEAS = np.vstack([
    VERTICES_CARTESIANAS,
    np.ones(VERTICES_CARTESIANAS.shape[1])
])

# Conexões das Arestas (índices dos vértices)
ARESTAS = [
    (0, 2), (0, 3), (0, 4), (0, 5),
    (1, 2), (1, 3), (1, 4), (1, 5),
    (4, 2), (4, 3), (5, 2), (5, 3)
]

# Dimensões do Viewport
VIEWPORT_WIDTH = 640
VIEWPORT_HEIGHT = 480
VIEWPORT_MIN = np.array([0, 0])
VIEWPORT_MAX = np.array([VIEWPORT_WIDTH, VIEWPORT_HEIGHT])

# --- Funções de Entrada e Validação do Usuário ---

def obter_coordenadas(prompt: str) -> np.ndarray:
    '''
    Solicita e retorna uma coordenada 3D do usuário.

    Entrada:
        prompt: A mensagem a ser exibida ao usuário.

    Saída:
        Um array com as coordenadas 3D [x, y, z].
    '''
    while True:
        try:
            coords_str = input(prompt)
            coords = list(map(float, coords_str.split())) # Separa por qualquer espaço em branco e converte para float
            if len(coords) != 3:
                raise ValueError("Por favor, insira exatamente 3 valores separados por espaços.")
            return np.array(coords)
        except ValueError as e:
            print(f"Entrada inválida. Erro: {e}")

def calcular_normal(P1: np.ndarray, P2: np.ndarray, P3: np.ndarray) -> np.ndarray:
    '''
    Calcula e retorna o vetor normal normalizado do plano definido por P1, P2 e P3.

    Retorna o vetor zero [0., 0., 0.] se os pontos forem colineares.

    Entrada:
        P1, P2, P3: Vetores 3D que definem o plano.

    Saída:
        O vetor normal normalizado.
    '''
    V12 = P1 - P2
    V32 = P3 - P2
    N = np.cross(V12, V32)
    
    norm_N = np.linalg.norm(N)
    
    # Se a norma for muito próxima de zero, os pontos são colineares
    if norm_N < 1e-9:
        return np.array([0., 0., 0.])
    
    return N / norm_N

def obter_dados_do_cenario():
    '''
    Coleta interativamente C, P1, P2, P3 e R0, validando a geometria para evitar
    projeções indefinidas (divisão por zero).

    Saída:
        Tupla de (C_PV, P1, P2, P3, R0_PONTO, N_NORMAL)
        C_PV: Ponto de Vista.
        P1, P2, P3: Pontos que definem o Plano de Projeção.
        R0_PONTO: Um ponto sobre o plano (igual a P1).
        N_NORMAL: O vetor normal normalizado do plano.
    '''
    print("\n--- Configuração da Projeção ---")
    R0_PONTO = None
    N_NORMAL = None
    
    # Obtem P1, P2, P3 e checa colinearidade
    while N_NORMAL is None or np.linalg.norm(N_NORMAL) < 1e-9:
        print("\n[Passo 1/2: Definir Plano de Projeção]")
        print("\n[Coordenadas de Exemplo Pronto] \n[00 00 -03] \n[00 02 -03] \n[02 00 -03]")
        print("\nPara inserir as coordenas, basta digitar os 3 números, com espaçamento e sem '[' ou ']'")
        print("Exemplo -> 00 00 00\n")
        P1 = obter_coordenadas("Insira as coordenadas do Ponto P1: ")
        P2 = obter_coordenadas("Insira as coordenadas do Ponto P2: ")
        P3 = obter_coordenadas("Insira as coordenadas do Ponto P3: ")
        
        R0_PONTO = P1
        N_NORMAL = calcular_normal(P1, P2, P3)
        
        if np.linalg.norm(N_NORMAL) < 1e-9:
            print("ERRO: Os pontos P1, P2 e P3 são colineares ou coincidentes. Insira novos pontos que definam um plano.")
            N_NORMAL = None 
        else:
            print("Plano de Projeção definido com sucesso.")
    
    # Obtem C e checa se está no plano
    C_PV = None
    d_check = 0 
    
    while C_PV is None or np.abs(d_check) < 1e-9:
        print("\n[Passo 2/2: Definir Ponto de Vista (PV)]")
        print("\n[Coordenadas de Ponto de Vista de Exemplo Pronto]")
        print("[02 02 10]\n")
        C_PV = obter_coordenadas("Insira as coordenadas do Ponto de Vista C: ")
        
        # Recalcula d0, d1, d com o novo C_PV e o N_NORMAL já validado
        d0, d1, d_check = calcular_parametros_perspectiva(C_PV, N_NORMAL, R0_PONTO)
        
        # O Ponto de Vista C NÃO pode estar no plano (d = 0), pois causaria divisão por zero
        if np.abs(d_check) < 1e-9:
            print(
                "ERRO CRÍTICO: O Ponto de Vista C está no Plano de Projeção! "
                "A projeção não é possível. Insira um novo PV."
            )
            C_PV = None 
        else:
            print("Ponto de Vista validado.")

    return C_PV, P1, P2, P3, R0_PONTO, N_NORMAL

# --- Funções Matemáticas (Projeção) ---

def calcular_parametros_perspectiva(C: np.ndarray, N: np.ndarray, R0: np.ndarray) -> tuple[float, float, float]:
    '''
    Calcula d0, d1 e d
    d0 = R0 . N (distância do plano à origem)
    d1 = C . N (distância do PV à origem, na direção N)
    d = d0 - d1 (distância do PV ao plano, na direção N)

    Entrada:
        C: Ponto de Vista
        N: Vetor normal normalizado do plano
        R0: Um ponto no plano de projeção

    Saída:
        Tupla de (d0, d1, d)
    '''
    d0 = np.dot(R0, N)
    d1 = np.dot(C, N)
    d = d0 - d1
    return d0, d1, d

def criar_matriz_perspectiva(C: np.ndarray, N: np.ndarray, d: float, d0: float, d1: float) -> np.ndarray:
    '''
    Cria a matriz de Projeção Perspectiva Mper (4x4)

    Entrada:
        C: Ponto de Vista (a, b, c)
        N: Vetor normal normalizado do plano (Nx, Ny, Nz)
        d: d0 - d1
        d0: R0 . N
        d1: C . N

    Saída:
        A matriz de projeção 4x4 
    '''
    a, b, c = C
    Nx, Ny, Nz = N
    
    # Matriz Mper conforme a fórmula da Projeção Perspectiva
    Mper = np.array([
        [d + a*Nx, a*Ny, a*Nz, -a*d0],
        [b*Nx, d + b*Ny, b*Nz, -b*d0],
        [c*Nx, c*Ny, d + c*Nz, -c*d0],
        [Nx, Ny, Nz, -d1] 
    ])
    
    return Mper

def projetar_para_cartesianas(Mper: np.ndarray, Mobjeto_homogenea: np.ndarray) -> np.ndarray:
    '''
    Aplica a projeção (Mper) e converte para coordenadas cartesianas 2D (XC, YC)
    dividindo por W' (a quarta coordenada homogênea).

    Entrada:
        Mper: Matriz de Projeção Perspectiva 4x4.
        Mobjeto_homogenea: Matriz dos vértices do objeto (4xN), em coordenadas homogêneas.

    Saída:
        Matriz (2xN) com as coordenadas cartesianas projetadas [XC, YC].
    '''
    Mob_projetado_homogenea = Mper @ Mobjeto_homogenea

    X_prime = Mob_projetado_homogenea[0, :]
    Y_prime = Mob_projetado_homogenea[1, :]
    W_prime = Mob_projetado_homogenea[3, :]

    # Tratamento de W' ≈ 0 para evitar divisão por zero
    # Se W' é zero, o ponto foi projetado para o infinito 
    # W' é substituido por um valor pequeno e diferente de zero, 
    # o que resultará em coordenadas muito grandes, que serão ignoradas/clipadas 
    # na fase de desenho ou transformação Janela-Viewport
    W_prime_zero = np.abs(W_prime) < 1e-9
    W_prime[W_prime_zero] = np.where(W_prime[W_prime_zero] >= 0, 1e-9, -1e-9)

    XC = X_prime / W_prime
    YC = Y_prime / W_prime
    
    return np.vstack([XC, YC])

# --- Transformação Janela-Viewport ---

def transformar_janela_viewport(Mobj_WCS_Projetado: np.ndarray, viewport_min: np.ndarray, viewport_max: np.ndarray) -> np.ndarray:
    '''
    Cria a matriz Tjv com centralização, preservando o Aspect Ratio,
    transformando coordenadas de Projeção (WCS) para Dispositivo (PDCS)

    Entrada:
        Mobj_WCS_Projetado: Matriz (2xN) com as coordenadas projetadas (XC, YC)
        viewport_min: Coordenadas mínimas do Viewport [U_min, V_min]
        viewport_max: Coordenadas máximas do Viewport [U_max, V_max]

    Saída:
        Matriz (2xN) com as coordenadas de tela (PDCS).
    '''
    U_min, V_min = viewport_min
    U_max, V_max = viewport_max
    
    X_min = np.min(Mobj_WCS_Projetado[0, :])
    X_max = np.max(Mobj_WCS_Projetado[0, :])
    Y_min = np.min(Mobj_WCS_Projetado[1, :])
    Y_max = np.max(Mobj_WCS_Projetado[1, :])
    
    # Tratamento de erro: se todos os pontos colapsaram em um único ponto ou linha/coluna
    X_range = X_max - X_min
    Y_range = Y_max - Y_min

    # Calcula fatores de escala base (S_x_base, S_y_base)
    # Evita erro de divisão por zero e trata o caso de objeto colapsado/invisível
    S_x_base = (U_max - U_min) / X_range if np.abs(X_range) > 1e-9 else 0.0
    S_y_base = (V_max - V_min) / Y_range if np.abs(Y_range) > 1e-9 else 0.0

    if S_x_base == 0.0 and S_y_base == 0.0:
        # Se as duas bases são zero, o objeto é um ponto ou não é visível; retorna zeros
        return np.zeros((2, Mobj_WCS_Projetado.shape[1]))

    # Escolhe o menor fator de escala para preservar o Aspect Ratio
    S_new = min(S_x_base, S_y_base) if S_x_base != 0.0 and S_y_base != 0.0 else max(S_x_base, S_y_base)

    Sx_final = S_new
    Sy_final = S_new

    # Cálculo do espaçamento para centralização
    U_new_range = S_new * X_range
    V_new_range = S_new * Y_range
    
    U_espacamento = (U_max - U_new_range) / 2
    V_espacamento = (V_max - V_new_range) / 2
    
    # Fatores de Deslocamento (T_x, T_y)
    # T_x: U_min + Espaçamento - (Escala * X_min)
    # T_y: V_min + Espaçamento + (Escala * Y_max)
    T_x = U_min + U_espacamento - Sx_final * X_min
    T_y = V_min + V_espacamento + Sy_final * Y_max 

    # Montagem da matriz Tjv (3x3)
    Tjv = np.array([
        [Sx_final, 0, T_x],
        [0, -Sy_final, T_y], 
        [0, 0, 1]
    ])
    
    Mobj_homogenea_2D = np.vstack([Mobj_WCS_Projetado, np.ones(Mobj_WCS_Projetado.shape[1])])
    Mobj_DCS = Tjv @ Mobj_homogenea_2D

    # Retorna apenas as coordenadas x e y de tela
    return Mobj_DCS[:2, :]

# --- Pipeline Principal e Visualização Pygame ---

def pipeline_completo(C_PV: np.ndarray, N_NORMAL: np.ndarray, R0_PONTO: np.ndarray, Mobjeto_homogenea: np.ndarray, viewport_min: np.ndarray, viewport_max: np.ndarray) -> np.ndarray:
    '''
    Executa o pipeline completo de Projeção Perspectiva: 
    Cálculo de parâmetros -> Matriz de Projeção -> Projeção Cartesiana -> Transformação Viewport.

    Retorna None se a projeção não for possível (C no plano).

    Entrada:
        C_PV: Ponto de Vista.
        N_NORMAL: Vetor normal do plano.
        R0_PONTO: Ponto no plano.
        Mobjeto_homogenea: Vértices do objeto em coordenadas homogêneas.
        viewport_min, viewport_max: Limites do viewport.

    Saida:
        Matriz (2xN) com as coordenadas de tela (PDCS) ou None.
    '''
    
    # Cálculo dos Parâmetros Perspectivos (d0, d1, d)
    d0, d1, d = calcular_parametros_perspectiva(C_PV, N_NORMAL, R0_PONTO)

    # Verificação de erro, embora já tratada na obtenção dos dados, para robustez
    if np.abs(d) < 1e-9:
        print("Erro: O Ponto de Vista C está no Plano de Projeção. Projeção impossível.")
        return None

    # Criação e aplicação da Matriz Perspectiva
    Mper = criar_matriz_perspectiva(C_PV, N_NORMAL, d, d0, d1)
    Mobj_WCS_Projetado = projetar_para_cartesianas(Mper, Mobjeto_homogenea)

    # Transformação Janela-Viewport
    Mobj_DCS = transformar_janela_viewport(Mobj_WCS_Projetado, viewport_min, viewport_max)
    
    return Mobj_DCS

def desenhar_objeto_pygame(screen: pygame.Surface, Mobj_DCS: np.ndarray, arestas: list[tuple[int, int]], color: tuple[int, int, int] = (255, 255, 255)):
    '''
    Desenha as arestas e vértices do objeto na tela Pygame.
    '''
    coords_pixels = Mobj_DCS.astype(int).T 
    
    for p1_idx, p2_idx in arestas:
        p1 = coords_pixels[p1_idx]
        p2 = coords_pixels[p2_idx]
        
        # O bloco try/except é mantido, pois a transformação Viewport não garante que
        # os pontos caibam em um inteiro ou no buffer da tela (apesar do recorte
        # do Pygame), especialmente se pontos no infinito causarem coordenadas extremas.
        try:
            pygame.draw.line(screen, color, p1, p2, 2)
        except ValueError: 
            # Captura a exceção se as coordenadas forem muito grandes (ex: overflow de int)
            continue
        
    for p in coords_pixels:
        try:
            pygame.draw.circle(screen, color, p, 3)
        except ValueError:
            continue


if __name__ == '__main__':
    # Executa os doctests para as funções matemáticas
    doctest.testmod(verbose=False) 

    # Coleta os dados do cenário
    C_PV, P1, P2, P3, R0_PONTO, N_NORMAL = obter_dados_do_cenario()

    # Executa o pipeline para obter as coordenadas de tela
    DCS_COORD = pipeline_completo(
        C_PV, N_NORMAL, R0_PONTO, VERTICES_HOMOGENEAS, VIEWPORT_MIN, VIEWPORT_MAX
    )
    
    if DCS_COORD is None:
        print("Não foi possível gerar as coordenadas de tela. Encerrando.")
        sys.exit(1) 

    # Inicializa Pygame e Visualização
    pygame.init()
    screen = pygame.display.set_mode((VIEWPORT_WIDTH, VIEWPORT_HEIGHT))
    pygame.display.set_caption("Projeção Perspectiva Cônica")
    
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    
    print("\nO objeto projetado está sendo exibido na janela Pygame.")

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        screen.fill(BLACK)
        desenhar_objeto_pygame(screen, DCS_COORD, ARESTAS, color=WHITE)
        pygame.display.flip()

    pygame.quit()