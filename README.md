# Visualização Projetiva - Perspectiva Cônica

Este projeto implementa um sistema de visualização gráfica baseado em **Projeção Perspectiva Cônica**, desenvolvido como requisito para a disciplina de Computação Gráfica (2025).

O software realiza todo o pipeline gráfico matemático, desde a definição do mundo 3D até a rasterização na tela, incluindo a transformação Janela-Viewport com preservação de razão de aspecto.

## Autores
* **Eduardo Angelo Rozada Minholi** - RA 134932
* **Nuno Miguel Mendonça Abilio** - RA 132830

## Funcionalidades

1.  **Definição Interativa de Cenário**: O usuário insere as coordenadas para definir o Plano de Projeção (3 pontos) e o Ponto de Vista (C).
2.  **Cálculo Vetorial Automático**: Cálculo da normal do plano e validação de colinearidade.
3.  **Pipeline de Projeção**:
    * Cálculo dos parâmetros de distância ($d0, d1, d$).
    * Construção da Matriz de Perspectiva 4x4.
    * Divisão Homogênea.
4.  **Transformação de Viewport**: Mapeamento das coordenadas normalizadas para coordenadas de dispositivo (tela), com centralização automática e preservação do *aspect ratio*.
5.  **Renderização**: Visualização *wireframe* de um Octaedro usando a biblioteca `pygame`.

## Pré-requisitos

Para executar este projeto, você precisa ter o **Python 3.x** instalado. Além disso, as seguintes bibliotecas são necessárias:

* `numpy` (Para operações matriciais e vetoriais)
* `pygame` (Para a criação da janela e desenho das linhas)

### Instalação das dependências

Abra o terminal e execute:

```bash
pip install numpy pygame