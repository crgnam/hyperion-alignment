function [outline_out] = get_arc(outline,start,N)
    if size(outline,2)-start < N
        outline_out = outline(:,start:end);
        outline_out = [outline_out, outline(:,1:(N-size(outline_out,2)))];
    else
       outline_out = outline(:,start:start+N-1);
    end
end