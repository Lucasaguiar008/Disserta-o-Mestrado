function vetorAleatorio = gerarVetorAleatorioSemRepeticao(n, limite)
    % n: número de elementos no vetor
    % limite: limite superior para os números aleatórios
    
    % Verificar se n é maior que o limite
    if n > limite
        error('O número de elementos no vetor não pode ser maior que o limite.');
    end
    
    % Gerar uma permutação aleatória de 1 até o limite
    permutacao = randperm(limite, n);
    
    % O vetor aleatório é a permutação resultante
    vetorAleatorio = permutacao;
end
