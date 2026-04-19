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

  var classics = [
    { line: '会当凌绝顶，一览众山小。', source: '杜甫《望岳》' },
    { line: '海上生明月，天涯共此时。', source: '张九龄《望月怀远》' },
    { line: '大江东去，浪淘尽，千古风流人物。', source: '苏轼《念奴娇·赤壁怀古》' },
    { line: '明月别枝惊鹊，清风半夜鸣蝉。', source: '辛弃疾《西江月》' },
    { line: '落霞与孤鹜齐飞，秋水共长天一色。', source: '王勃《滕王阁序》' },
    { line: '天行健，君子以自强不息。', source: '《周易》' },
    { line: '路漫漫其修远兮，吾将上下而求索。', source: '屈原《离骚》' },
    { line: '长风破浪会有时，直挂云帆济沧海。', source: '李白《行路难》' }
  ];

  function getDailySeed() {
    var d = new Date();
    return d.getFullYear() * 1000 + (d.getMonth() + 1) * 50 + d.getDate();
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

    wrap.appendChild(line);
    wrap.appendChild(source);
    siteInfo.appendChild(wrap);

    return wrap;
  }

  function preloadImage(url, onLoad) {
    var img = new Image();
    img.onload = function () {
      onLoad(url);
    };
    img.src = url;
  }

  function applyWallpaper(url) {
    var header = getHeaderElement();
    if (!header) {
      return;
    }

    header.style.backgroundImage = 'linear-gradient(120deg, rgba(3, 17, 36, 0.52), rgba(5, 24, 52, 0.36)), url(' + url + ')';
    header.style.backgroundSize = 'cover';
    header.style.backgroundPosition = 'center center';
    header.style.backgroundRepeat = 'no-repeat';
  }

  function applyClassic(item) {
    var wrap = createPoetryNode();
    if (!wrap) {
      return;
    }

    var lineNode = wrap.querySelector('.jj-hero-poetry-line');
    var sourceNode = wrap.querySelector('.jj-hero-poetry-source');
    lineNode.textContent = item.line;
    sourceNode.textContent = item.source;
  }

  function startDynamicHome() {
    if (!isHomePage()) {
      return;
    }

    var seed = getDailySeed();
    var wallpaperIndex = seed % wallpapers.length;
    var classicIndex = seed % classics.length;

    preloadImage(wallpapers[wallpaperIndex], applyWallpaper);
    applyClassic(classics[classicIndex]);

    window.setInterval(function () {
      wallpaperIndex = (wallpaperIndex + 1) % wallpapers.length;
      classicIndex = (classicIndex + 1) % classics.length;
      preloadImage(wallpapers[wallpaperIndex], applyWallpaper);
      applyClassic(classics[classicIndex]);
    }, 45000);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', startDynamicHome);
  } else {
    startDynamicHome();
  }
})();
