(function () {
  'use strict';

  var STORAGE_KEY = 'jj-code-theme';
  var ROOT_ATTR = 'data-code-theme';

  function getStoredTheme() {
    try {
      return window.localStorage.getItem(STORAGE_KEY) || 'light';
    } catch (error) {
      return 'light';
    }
  }

  function setStoredTheme(theme) {
    try {
      window.localStorage.setItem(STORAGE_KEY, theme);
    } catch (error) {
      return;
    }
  }

  function applyTheme(theme) {
    document.documentElement.setAttribute(ROOT_ATTR, theme);
  }

  function hasCodeBlocks() {
    return Boolean(document.querySelector('#article-container pre, #article-container figure.highlight'));
  }

  function getButtonLabel(theme) {
    return theme === 'dark' ? '代码暗色' : '代码亮色';
  }

  function updateButton(button, theme) {
    button.textContent = getButtonLabel(theme);
    button.setAttribute('aria-label', '切换代码块亮暗主题');
    button.setAttribute('title', '切换代码块亮暗主题');
  }

  function ensureButton() {
    var existing = document.querySelector('.jj-code-theme-toggle');
    if (!hasCodeBlocks()) {
      if (existing) {
        existing.remove();
      }
      return;
    }

    var button = existing;
    if (!button) {
      button = document.createElement('button');
      button.className = 'jj-code-theme-toggle';
      button.type = 'button';
      document.body.appendChild(button);
    }

    if (!button.dataset.bound) {
      button.dataset.bound = '1';
      button.addEventListener('click', function () {
        var nextTheme = document.documentElement.getAttribute(ROOT_ATTR) === 'dark' ? 'light' : 'dark';
        applyTheme(nextTheme);
        setStoredTheme(nextTheme);
        updateButton(button, nextTheme);
      });
    }

    updateButton(button, getStoredTheme());
  }

  function initCodeTheme() {
    applyTheme(getStoredTheme());
    ensureButton();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initCodeTheme);
  } else {
    initCodeTheme();
  }

  document.addEventListener('pjax:complete', initCodeTheme);
})();