# 📡 IoT IR Automation System (v5.2.0)

Sistema de controle remoto universal baseado em **ESP32**, desenvolvido para automação de dispositivos infravermelhos (Ar-condicionado, TVs, Ventiladores). Este projeto destaca-se pelo uso de algoritmos estatísticos para garantir a fidelidade do sinal capturado.

## 🚀 Diferenciais Técnicos (v5.2.0)

### 1. Algoritmo de Tripla Captura Inteligente
Para mitigar ruídos de interferência (comuns em ambientes com lâmpadas LED ou fluorescentes), o sistema realiza 3 amostragens consecutivas do sinal RAW. 
- Utiliza o **Cálculo de Desvio Padrão** para comparar as capturas.
- Seleciona automaticamente a amostra com a menor variância (mais estável).
- **Resultado:** Maior taxa de sucesso no acionamento de aparelhos antigos ou genéricos.

### 2. Suporte Nativo de Protocolos
Em vez de depender apenas de sinais RAW (que consomem muita memória), o sistema integra bibliotecas para suporte direto:
- **Hitachi** (Padrão utilizado em instituições como o **SENAC**).
- **Midea / Elgin** (Protocolos de 48-bits).
- **Samsung / LG / Sony** (Protocolos de 32-bits).

## 🛠️ Stack Tecnológica
- **Linguagem:** C++ (Framework Arduino)
- **Hardware:** ESP32 (Dual Core), LED Emissor IR, Receptor VS1838B.
- **Persistência:** Uso de **NVS (Non-Volatile Storage)** para salvar sinais aprendidos via chaves Hash (CRC32).
- **Conectividade:** Multi-Network WiFi (STA/AP) com suporte a **mDNS** (`http://ir-remote.local`).
- **DevOps:** Atualização de firmware via **OTA (Over-The-Air)**.

## 📂 Estrutura do Projeto
- `src/main.cpp`: Core do sistema, gerenciamento de rotas API e lógica de processamento IR.
- `include/`: Definições de hardware e cabeçalhos.
- `lib/`: Bibliotecas customizadas de protocolos AC.

---

## 👨‍💻 Sobre o Desenvolvedor

---

## Sobre o Desenvolvedor

<table border="0" cellpadding="0" cellspacing="0" width="100%">
  <tr>
    <td width="150px" align="center" valign="top">
      <img src="https://github.com/user-attachments/assets/00bdbcbd-e331-4104-bb82-f39c09073672" alt="Edinaldo Santos" width="130" height="130" style="border-radius: 50%; display: block;">
    </td>
    <td valign="top" style="padding-left: 20px;">
      <h3 style="margin-top: 0;">Edinaldo Santos de Almeida</h3>
      <p style="margin: 0;">
        Técnico em Desenvolvimento de Sistemas<br>
        <strong>SENAC Franca/SP</strong>
      </p>
      <p style="margin-top: 15px;">
        Foco em Programação Back-End e Sistemas Embarcados IoT
      </p>
    </td>
  </tr>
</table>

---
> *Este repositório serve como demonstração de competências em integração de Hardware/Software, tratamento de sinais e desenvolvimento de APIs em sistemas embarcados.*
