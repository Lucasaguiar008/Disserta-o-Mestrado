function salvarStringNoArquivo(minhaString, nomeArquivo)
    % Abre o arquivo para escrita
    fid = fopen(nomeArquivo, 'w');

    % Verifica se o arquivo foi aberto corretamente
    if fid == -1
        error('Erro ao abrir o arquivo para escrita.');
    end

    % Escreve a string no arquivo
    fprintf(fid, '%s\n', minhaString);

    % Fecha o arquivo
    fclose(fid);

    disp(['String salva com sucesso no arquivo: ' nomeArquivo]);
end
