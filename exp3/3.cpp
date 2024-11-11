#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <queue>
#include <vector>

// 定义二叉树节点
struct BinTreeNode {
    char ch; // 节点代表的字符（对于叶子节点）
    int freq; // 节点的频率
    BinTreeNode *left; // 左子节点
    BinTreeNode *right; // 右子节点

    // 构造函数（叶子节点）
    BinTreeNode(char character, int frequency) : ch(character), freq(frequency), left(nullptr), right(nullptr) {}

    // 构造函数（内部节点）
    BinTreeNode(int frequency, BinTreeNode *l, BinTreeNode *r) : ch('\0'), freq(frequency), left(l), right(r) {}
};

// 比较函数，用于优先队列
struct Compare {
    bool operator()(BinTreeNode *l, BinTreeNode *r) {
        return l->freq > r->freq;
    }
};

// 位图类，用于存储位序列
class Bitmap {
private:
    std::vector<bool> bits;

public:
    void addBit(bool bit) {
        bits.push_back(bit);
    }

    void removeLastBit() {
        if (!bits.empty()) {
            bits.pop_back();
        }
    }

    Bitmap copy() {
        Bitmap new_bitmap;
        new_bitmap.bits = bits;
        return new_bitmap;
    }

    std::string toString() {
        std::string s;
        for (bool bit : bits) {
            s += bit ? '1' : '0';
        }
        return s;
    }
};

// 递归函数，生成编码
void generateCodes(BinTreeNode *node, Bitmap &current_code, std::map<char, Bitmap> &codes) {
    if (!node) return;
    // 如果是叶子节点，保存编码
    if (!node->left && !node->right) {
        codes[node->ch] = current_code.copy();
        return;
    }
    // 遍历左子树
    current_code.addBit(0);
    generateCodes(node->left, current_code, codes);
    current_code.removeLastBit();
    // 遍历右子树
    current_code.addBit(1);
    generateCodes(node->right, current_code, codes);
    current_code.removeLastBit();
}

int main() {
    // 获取演讲文本（为简洁，此处使用演讲的部分内容）
    std::string speech = R"(I have a dream that one day this nation will rise up and live out the true meaning of its creed:
    'We hold these truths to be self-evident; that all men are created equal.')";

    // 将文本转换为小写，并只保留字母
    std::string text;
    for (char c : speech) {
        if (isalpha(c)) {
            text += tolower(c);
        }
    }

    // 统计每个字母的频率
    std::map<char, int> freq_map;
    for (char c : text) {
        freq_map[c]++;
    }

    // 创建初始的叶子节点列表
    std::priority_queue<BinTreeNode*, std::vector<BinTreeNode*>, Compare> heap;
    for (auto pair : freq_map) {
        heap.push(new BinTreeNode(pair.first, pair.second));
    }

    // 构建 Huffman 树
    while (heap.size() > 1) {
        BinTreeNode *left = heap.top();
        heap.pop();
        BinTreeNode *right = heap.top();
        heap.pop();
        BinTreeNode *merged = new BinTreeNode(left->freq + right->freq, left, right);
        heap.push(merged);
    }

    // 最终的 Huffman 树
    BinTreeNode *huffman_tree = heap.top();

    // 生成字符的 Huffman 编码
    std::map<char, Bitmap> huffman_codes;
    Bitmap current_code;
    generateCodes(huffman_tree, current_code, huffman_codes);

    // 对单词进行编码
    auto encodeWord = [&](const std::string &word) {
        std::string encoded;
        for (char c : word) {
            char lower_c = tolower(c);
            if (huffman_codes.find(lower_c) != huffman_codes.end()) {
                encoded += huffman_codes[lower_c].toString();
            } else {
                std::cout << "Character '" << c << "' not in Huffman tree.\n";
                return std::string();
            }
        }
        return encoded;
    };

    // 编码单词 "dream"
    std::string word1 = "dream";
    std::string encoded_word1 = encodeWord(word1);
    std::cout << "Huffman encoding for '" << word1 << "': " << encoded_word1 << std::endl;

    // 编码其他单词，例如 "equal"
    std::string word2 = "equal";
    std::string encoded_word2 = encodeWord(word2);
    std::cout << "Huffman encoding for '" << word2 << "': " << encoded_word2 << std::endl;

    return 0;
}
