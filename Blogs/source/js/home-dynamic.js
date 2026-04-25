(function () {
  'use strict';

  function isHomePage() {
    var path = window.location.pathname;
    return path === '/' || path === '/index.html';
  }

  function getHeaderElement() {
    return document.getElementById('page-header');
  }

  function getSiteInfoElement() {
    return document.getElementById('site-info');
  }

  var wallpapers = [
    'https://images.unsplash.com/photo-1506744038136-46273834b3fb?auto=format&fit=crop&w=1920&q=80',
    'https://images.unsplash.com/photo-1472214103451-9374bd1c798e?auto=format&fit=crop&w=1920&q=80',
    'https://images.unsplash.com/photo-1469474968028-56623f02e42e?auto=format&fit=crop&w=1920&q=80',
    'https://images.unsplash.com/photo-1441974231531-c6227db76b6e?auto=format&fit=crop&w=1920&q=80',
    'https://images.unsplash.com/photo-1493244040629-496f6d136cc3?auto=format&fit=crop&w=1920&q=80',
    'https://images.unsplash.com/photo-1500530855697-b586d89ba3ee?auto=format&fit=crop&w=1920&q=80'
  ];

  // 本地备用诗词池，API 失败时降级使用（覆盖多朝代经典，供 换一首 轮换）
  var fallbackPoems = [
    { line: '等闲识得东风面，万紫千红总是春。', source: '朱熹《春日》' },
    { line: '落霞与孤鹜齐飞，秋水共长天一色。', source: '王勃《滕王阁序》' },
    { line: '接天莲叶无穷碧，映日荷花别样红。', source: '杨万里《晓出净慈寺送林子方》' },
    { line: '晴空一鹤排云上，便引诗情到碧霄。', source: '刘禹锡《秋词》' },
    { line: '忽如一夜春风来，千树万树梨花开。', source: '岑参《白雪歌送武判官归京》' },
    { line: '千山鸟飞绝，万径人踪灭。', source: '柳宗元《江雪》' },
    { line: '停车坐爱枫林晚，霜叶红于二月花。', source: '杜牧《山行》' },
    { line: '但愿人长久，千里共婵娟。', source: '苏轼《水调歌头》' },
    { line: '会当凌绝顶，一览众山小。', source: '杜甫《望岳》' },
    { line: '举头望明月，低头思故乡。', source: '李白《静夜思》' },
    { line: '春蚕到死丝方尽，蜡炬成灰泪始干。', source: '李商隐《无题》' },
    { line: '采菊东篱下，悠然见南山。', source: '陶渊明《饮酒》' },
    { line: '人生自古谁无死，留取丹心照汗青。', source: '文天祥《过零丁洋》' },
    { line: '海内存知己，天涯若比邻。', source: '王勃《送杜少府之任蜀州》' },
    { line: '欲穷千里目，更上一层楼。', source: '王之涣《登鹳雀楼》' },
    { line: '烽火连三月，家书抵万金。', source: '杜甫《春望》' },
    { line: '大漠孤烟直，长河落日圆。', source: '王维《使至塞上》' },
    { line: '黄河之水天上来，奔流到海不复回。', source: '李白《将进酒》' },
    { line: '问渠那得清如许，为有源头活水来。', source: '朱熹《观书有感》' },
    { line: '不识庐山真面目，只缘身在此山中。', source: '苏轼《题西林壁》' },
    { line: '春风得意马蹄疾，一日看尽长安花。', source: '孟郊《登科后》' },
    { line: '疏影横斜水清浅，暗香浮动月黄昏。', source: '林逋《山园小梅》' },
    { line: '醉卧沙场君莫笑，古来征战几人回。', source: '王翰《凉州词》' },
    { line: '桃花潭水深千尺，不及汪伦送我情。', source: '李白《赠汪伦》' },
    { line: '少壮不努力，老大徒伤悲。', source: '《长歌行》' },
    { line: '春色满园关不住，一枝红杏出墙来。', source: '叶绍翁《游园不值》' },
    { line: '沉舟侧畔千帆过，病树前头万木春。', source: '刘禹锡《酬乐天扬州初逢席上见赠》' },
    { line: '山重水复疑无路，柳暗花明又一村。', source: '陆游《游山西村》' },
    { line: '天生我材必有用，千金散尽还复来。', source: '李白《将进酒》' },
    { line: '安能摧眉折腰事权贵，使我不得开心颜。', source: '李白《梦游天姥吟留别》' },
    { line: '横眉冷对千夫指，俯首甘为孺子牛。', source: '鲁迅《自嘲》' },
    { line: '宝剑锋从磨砺出，梅花香自苦寒来。', source: '《警世贤文》' }
  ];

  // 备用诗词轮换索引（每次点击换一首时递增，不依赖固定种子）
  var fallbackIndex = Math.floor(Math.random() * 32);

  function getDailySeed() {
    var d = new Date();
    return d.getFullYear() * 1000 + (d.getMonth() + 1) * 50 + d.getDate();
  }

  // 调用今日诗词 API（接入 chinese-poetry 库，30 万+ 经典条目）
  // 加 _t 时间戳防止浏览器缓存；不传 withCredentials 避免 session 锁定同一首
  function fetchPoetryFromAPI(callback) {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', 'https://v2.jinrishici.com/one.json?_t=' + Date.now(), true);
    xhr.timeout = 5000;
    xhr.onreadystatechange = function () {
      if (xhr.readyState !== 4) return;
      if (xhr.status === 200) {
        try {
          var resp = JSON.parse(xhr.responseText);
          if (resp.status === 'success' && resp.data && resp.data.content) {
            var origin = resp.data.origin || {};
            var src = (origin.author ? origin.author : '') +
              (origin.title ? '《' + origin.title + '》' : '');
            callback({ line: resp.data.content, source: src });
            return;
          }
        } catch (e) {}
      }
      callback(null);
    };
    xhr.onerror = function () { callback(null); };
    xhr.ontimeout = function () { callback(null); };
    xhr.send();
  }

  // 备用诗词：每次调用递增索引，保证换一首每次不同
  function getFallbackPoem() {
    fallbackIndex = (fallbackIndex + 1) % fallbackPoems.length;
    return fallbackPoems[fallbackIndex];
  }

  function createPoetryNode() {
    var existing = document.querySelector('.jj-hero-poetry');
    if (existing) {
      return existing;
    }

    var siteInfo = getSiteInfoElement();
    if (!siteInfo) {
      return null;
    }

    var wrap = document.createElement('div');
    wrap.className = 'jj-hero-poetry';

    var line = document.createElement('div');
    line.className = 'jj-hero-poetry-line';

    var source = document.createElement('div');
    source.className = 'jj-hero-poetry-source';

    var button = document.createElement('button');
    button.className = 'jj-hero-poetry-btn';
    button.type = 'button';
    button.textContent = '换一首';

    wrap.appendChild(line);
    wrap.appendChild(source);
    wrap.appendChild(button);
    siteInfo.appendChild(wrap);

    return wrap;
  }

  function applyWallpaper(url) {
    var header = getHeaderElement();
    if (!header) {
      return;
    }

    header.style.backgroundImage = 'linear-gradient(120deg, rgba(4, 13, 28, 0.72), rgba(10, 27, 54, 0.58)), url(' + url + ')';
    header.style.backgroundSize = 'cover';
    header.style.backgroundPosition = 'center center';
    header.style.backgroundRepeat = 'no-repeat';
  }

  function applyClassic(item) {
    var wrap = createPoetryNode();
    if (!wrap) return;
    wrap.querySelector('.jj-hero-poetry-line').textContent = item.line;
    wrap.querySelector('.jj-hero-poetry-source').textContent = item.source;
  }

  // 从今日诗词 API 获取，失败时降级本地备用
  function loadPoetry() {
    var wrap = createPoetryNode();
    if (wrap) {
      var lineNode = wrap.querySelector('.jj-hero-poetry-line');
      if (lineNode) lineNode.textContent = '正在加载诗词…';
    }
    fetchPoetryFromAPI(function (item) {
      applyClassic(item || getFallbackPoem());
    });
  }

  function startDynamicHome() {
    if (!isHomePage()) return;

    var seed = getDailySeed();
    var wallpaperIndex = seed % wallpapers.length;

    // 直接应用壁纸，不等预加载，消除 seele.jpg 闪烁
    applyWallpaper(wallpapers[wallpaperIndex]);

    // 从中华诗词 API 动态加载诗词
    loadPoetry();

    var poetryWrap = createPoetryNode();
    if (poetryWrap) {
      var nextBtn = poetryWrap.querySelector('.jj-hero-poetry-btn');
      if (nextBtn && !nextBtn.dataset.bound) {
        nextBtn.dataset.bound = '1';
        nextBtn.addEventListener('click', function () {
          loadPoetry();
        });
      }
    }

    window.setInterval(function () {
      wallpaperIndex = (wallpaperIndex + 1) % wallpapers.length;
      applyWallpaper(wallpapers[wallpaperIndex]);
      loadPoetry();
    }, 45000);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', startDynamicHome);
  } else {
    startDynamicHome();
  }
})();
